from tkinter import (Tk, Toplevel, Canvas, Button, Label, Scale, Checkbutton, SUNKEN, RAISED, messagebox, FIRST,
                     LAST, BOTH, simpledialog, OptionMenu, StringVar, HORIZONTAL, VERTICAL, IntVar, DoubleVar,
                     BooleanVar, filedialog)
from tkinter.ttk import Progressbar, Separator
from collections import namedtuple
from abc import ABC, abstractmethod
from copy import deepcopy
from numpy import array, linalg, dot

from Vector2D import Vec2D
from Floor import Floor
from Map import Map
from Constants import GlobalConstants


class Manager:
    # Manager class - handles UI and inputs
    # ui_fonts must store both the fonts and the sizes as strings and integers respectively
    FontTuple = namedtuple("FontTuple", "font size")
    # All font size decisions were made based of a window with a 1024 by 534 resolution and so must be scaled
    # appropriately for different resolutions
    FONT_REFERENCE_RES = Vec2D(1024, 534)
    # If the canvas is made the maximum possible size, the outer vertices are hidden by the edges of the canvas, so
    # every widget must be scaled appropriately
    CANVAS_SCALE = 0.9
    # Tool constants
    NONE = "N"
    CREATE_WALL = "W"
    FINISH_WALL = "W2"
    CREATE_LINK = "L"
    DELETE = "D"
    EDIT_WEIGHT = "E"
    CHANGE_DIRECTION = "C"
    BLOCK_PATH = "B"
    PLACE_START = "S"
    PLACE_GOAL = "G"
    FIND_PATH = "F"
    # Stage constants
    MAIN_MENU = 0
    MAP_CREATION = 1
    NAV_MESH_GENERATION = 2
    NAV_GRAPH_TWEAKING = 3
    PATHFINDING = 4
    RETURN = 5
    # Options constants
    DISABLE_WARNINGS = "Disable\nWarnings"
    PRIMARY_ALGORITHM = "Primary\nAlgorithm"
    LINK_ALGORITHM = "Link\nAlgorithm"
    NODES_PER_EDGE = "Nodes\nPer Edge"
    ENABLE_GRID = "Enable\nGrid"
    GRID_SIZE_X = "Grid\nSize X"
    GRID_SIZE_Y = "Grid\nSize Y"
    PRECOMPUTE_LINK_DIST = "Precomp\nLink Dist"
    THREAD_TARGET = "Thread\nTarget"
    LINK_WEIGHT = "Link\nWeight"
    SMOOTH_FINAL_PATH = "Smooth\nPath"

    DEFAULT_OPTIONS = {DISABLE_WARNINGS: False, PRIMARY_ALGORITHM: GlobalConstants.A_STAR,
                       LINK_ALGORITHM: GlobalConstants.DIJKSTRA, NODES_PER_EDGE: 1,
                       ENABLE_GRID: False, GRID_SIZE_X: 15, GRID_SIZE_Y: 9, PRECOMPUTE_LINK_DIST: False,
                       THREAD_TARGET: 8, LINK_WEIGHT: 1.0, SMOOTH_FINAL_PATH: False}
    # Canvas widget objects
    # ABC is the Abstract Base Class that all abstract base classes must inherit
    class AbstractCanvasWidget(ABC):

        @abstractmethod
        def draw_scaled(self, master, canvas):
            # Abstract method that must be overridden for each widget
            pass

    class LineCanvasWidget(AbstractCanvasWidget):
        def __init__(self, point1, point2, width, dash, arrow):
            self.__point1 = point1
            self.__point2 = point2
            self.__width = width
            self.__dash = dash
            self.__arrow = arrow

        def draw_scaled(self, master, canvas):
            canvas_point1 = master.map_to_canvas_space(self.__point1)
            canvas_point2 = master.map_to_canvas_space(self.__point2)
            # Multiply by root canvas area to keep fraction of canvas area covered by line constant
            # Minimum of width * 200 is to keep the line from having <1 width at low resolutions, and to make sure
            # thicker always stay appear noticeably thicker
            canvas_width = max(self.__width * master.min_canvas_scale, self.__width*500)
            canvas.create_line(canvas_point1.x, canvas_point1.y, canvas_point2.x, canvas_point2.y,
                               width=canvas_width, dash=self.__dash, arrow=self.__arrow)

    class CircleCanvasWidget(AbstractCanvasWidget):
        def __init__(self, position, radius, fill_colour, border_width):
            self.__position = position
            self.__radius = radius
            self.__fill_colour = fill_colour
            self.__border_width = border_width

        def draw_scaled(self, master, canvas):
            canvas_position = master.map_to_canvas_space(self.__position)
            canvas_radius = self.__radius * master.min_canvas_scale
            canvas_top_left = Vec2D.add(canvas_position, Vec2D(canvas_radius, canvas_radius))
            canvas_bottom_right = Vec2D.sub(canvas_position, Vec2D(canvas_radius, canvas_radius))
            canvas.create_oval(canvas_top_left.x, canvas_top_left.y, canvas_bottom_right.x, canvas_bottom_right.y,
                               fill=self.__fill_colour, width=self.__border_width)

    class TextCanvasWidget(AbstractCanvasWidget):
        def __init__(self, position, text, font, size):
            self.__position = position
            self.__text = text
            self.__font = font
            self.__size = size

        def draw_scaled(self, master, canvas):
            canvas_position = master.map_to_canvas_space(self.__position)
            text_ref = canvas.create_text(canvas_position.x, canvas_position.y, text=self.__text, font=self.__font %
                                          int(1 + self.__size * master.min_canvas_scale))
            x1, y1, x2, y2 = canvas.bbox(text_ref)
            canvas.create_rectangle(x1, y1, x2, y2, fill="#F0F0F0")
            canvas.tag_raise(text_ref)

    def __init__(self):
        self.__window = Tk()
        self.__window.title("GUI")
        self.__window.state("zoomed")
        self.resolution = Vec2D(self.__window.winfo_width(), self.__window.winfo_height())
        self.__window.bind("<Configure>", self.__resize)
        self.__widget_refs = dict()
        self.__widget_fonts = dict()
        self.__canvas = None
        self.__active_tool = self.NONE
        self.__prev_click_pos = None
        self.__shift_held = False
        self.__cancel = False
        self.__start_floor_num = 0
        self.__goal_floor_num = 0
        self.__start_pos = None
        self.__goal_pos = None
        self.__path = None
        self.__window.bind("<KeyPress>", self.key_down)
        self.__window.bind("<KeyRelease>", self.key_up)
        self.__current_stage = self.MAIN_MENU
        self.__current_floor_num = 0
        self.__map = Map()
        self.__canvas_widget_data = []
        self.__options = deepcopy(self.DEFAULT_OPTIONS)
        self.__new_options = None
        self.main_menu_ui()
        self.__window.mainloop()

    @property
    def resolution(self):
        return self.__resolution

    @property
    def __current_floor(self):
        return self.__map.floors[self.__current_floor_num]

    @property
    def __current_nav_mesh(self):
        return self.__map.nav_meshes[self.__current_floor_num]

    @resolution.setter
    def resolution(self, value):
        # Updating resolution must also update min_scale
        # Pre-computing min_scale here avoids recalculating it every time a new widget is added
        self.__resolution = value
        font_scales = Vec2D.component_wise_div(self.resolution, self.FONT_REFERENCE_RES)
        self.min_font_scale = min(font_scales.x, font_scales.y)
        canvas_scales = Vec2D.component_wise_div(Vec2D(self.resolution.x, self.resolution.y * 6 / 7),
                                                 GlobalConstants.FLOOR_RATIO)
        self.min_canvas_scale = min(canvas_scales.x, canvas_scales.y)

    def add_canvas_line(self, point1, point2, width=0.002, dash=(), arrow="none"):
        self.__canvas_widget_data.append(self.LineCanvasWidget(point1, point2, width, dash, arrow))
        self.__canvas_widget_data[-1].draw_scaled(self, self.__canvas)

    def add_canvas_circle(self, position, radius=0.015, fill_colour="", border_width=1):
        self.__canvas_widget_data.append(self.CircleCanvasWidget(position, radius, fill_colour, border_width))
        self.__canvas_widget_data[-1].draw_scaled(self, self.__canvas)

    def add_canvas_text(self, position, text, font="TkDefaultFont %s", size=0.015):
        # Note that text size is much smaller here than for the main UI buttons as it is scaled based of
        # min_canvas_scale
        self.__canvas_widget_data.append(self.TextCanvasWidget(position, text, font, size))
        self.__canvas_widget_data[-1].draw_scaled(self, self.__canvas)

    def __add_widget(self, widget_name, widget_obj, x_pos, y_pos, width, height, font, font_size):
        self.__widget_refs[widget_name] = widget_obj
        self.__widget_refs[widget_name].place(anchor="center", relx=x_pos, rely=y_pos, relwidth=width, relheight=height)
        self.__widget_fonts[widget_name] = self.FontTuple(font, font_size)
        self.__scale_widget(widget_name)

    def __resize(self, _):
        # Updates window resolution
        self.resolution = Vec2D(self.__window.winfo_width(), self.__window.winfo_height())
        for widget_name in self.__widget_refs.keys():
            # Scales text size of all widgets
            self.__scale_widget(widget_name)
        if self.__canvas is not None:
            # Resize canvas
            self.__canvas.config(width=self.resolution.x, height=self.resolution.y * 6 / 7)
            # Redraws all canvas elements
            self.refresh_canvas()

    def __scale_widget(self, widget_name):
        # Minimum font size of 1
        if not widget_name == "ProgressBar":
            self.__widget_refs[widget_name].configure(font=self.__widget_fonts[widget_name].font %
                                                      int(1 + self.__widget_fonts[widget_name].size *
                                                          self.min_font_scale))
        # Progress bar does not have text

    def __destroy_widgets(self):
        for widget_name in self.__widget_refs.keys():
            self.__widget_refs[widget_name].destroy()
        self.__widget_fonts.clear()
        self.__widget_refs.clear()

    def main_menu_ui(self):
        if self.__new_options is not None:
            self.__options = self.__new_options
            self.__new_options = None
            self.info("Setting changes applied!")
        self.__add_widget("NewMapButton", Button(self.__window, text="New Map", command=self.new_map,
                                                 bg="light grey"),
                          0.5, 5 / 14, 0.25, 1 / 6, "TkDefaultFont %s", 16)
        self.__add_widget("LoadMapButton", Button(self.__window, text="Load Map", command=self.load_map,
                                                  bg="light grey"),
                          0.5, 13 / 21, 0.25, 1 / 6, "TkDefaultFont %s", 16)
        self.__add_widget("OptionsButton", Button(self.__window, text="Options", command=self.open_options,
                                                  bg="light grey"),
                          7 / 128, 1 / 12, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget("MainMenuText", Label(self.__window, text="Main Menu"), 1 / 2, 1 / 12,
                          1 / 2, 1 / 7, "TkDefaultFont %s underline", 26)

    def new_map(self):
        self.__map = Map()
        self.next_stage()

    def load_map(self):
        file1_obj, file2_obj = self.open_file()
        if file1_obj is None:
            return
        else:
            self.__map = Map(data=file1_obj.read().split("\n"))
            self.next_stage()
            if file2_obj is not None:
                self.next_stage()
                self.__map.load_nav_graph_edits(file2_obj.read().split("\n"))
                file2_obj.close()
            file1_obj.close()
            self.update_canvas()

    def map_creation_ui(self):
        if self.__new_options is not None:
            self.__options = self.__new_options
            self.__new_options = None
            self.info("Setting changes applied!")
        self.__add_widget(self.CREATE_WALL, Button(self.__window, text="Create\nWall", command=lambda:
                                                   self.__set_tool(self.CREATE_WALL), bg="light grey"),
                          31 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget(self.CREATE_LINK, Button(self.__window, text="Create\nLink", command=lambda:
                                                   self.__set_tool(self.CREATE_LINK), bg="light grey"),
                          21 / 64, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget(self.DELETE, Button(self.__window, text="Delete", command=lambda:
                                              self.__set_tool(self.DELETE), bg="light grey"),
                          53 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)

    def nav_mesh_generation_ui(self):
        self.__add_widget("CancelButton", Button(self.__window, text="Cancel", command=self.cancel,
                                                 bg="light grey"),
                          122 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget("ProgressBar", Progressbar(self.__window, maximum=1.0, mode="determinate"),
                          1 / 2, 3 / 42, 3 / 8, 5 / 42, "TkDefaultFont %s", 12)

    def nav_mesh_generation_update_prog(self, complete):
        # Updates progress bar based on fraction complete and returns if nav mesh generation has been cancelled
        self.__widget_refs["ProgressBar"]["value"] = complete
        self.__window.update()
        if self.__cancel:
            return False
        else:
            return True

    def nav_graph_tweaking_ui(self):
        self.__add_widget(self.EDIT_WEIGHT, Button(self.__window, text="Edit\nWeight", command=lambda:
                                                   self.__set_tool(self.EDIT_WEIGHT), bg="light grey"),
                          31 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget(self.CHANGE_DIRECTION, Button(self.__window, text="Change\nDirection", command=lambda:
                                                        self.__set_tool(self.CHANGE_DIRECTION), bg="light grey"),
                          21 / 64, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget(self.BLOCK_PATH, Button(self.__window, text="Block\nPath", command=lambda:
                                                  self.__set_tool(self.BLOCK_PATH), bg="light grey"),
                          53 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)

    def pathfinding_ui(self):
        self.__add_widget(self.PLACE_START, Button(self.__window, text="Place\nStart", command=lambda:
                                                   self.__set_tool(self.PLACE_START), bg="light grey"),
                          31 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget(self.PLACE_GOAL, Button(self.__window, text="Place\nGoal", command=lambda:
                                                  self.__set_tool(self.PLACE_GOAL), bg="light grey"),
                          21 / 64, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget(self.FIND_PATH, Button(self.__window, text="Find\nPath", command=lambda:
                                                 self.find_path(), bg="light grey"),
                          53 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)

    def __place_standard_buttons(self):
        # Places the standard buttons used in each stage
        self.__add_widget("SaveButton", Button(self.__window, text="Save", command=self.save, bg="light grey"),
                          3 / 64, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget("OptionsButton", Button(self.__window, text="Options", command=self.open_options,
                                                  bg="light grey"),
                          17 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget("GoDownFloorButton", Button(self.__window, text="Go\nDown\nFloor", command=lambda:
                                                      self.change_floor(-1), bg="light grey"),
                          67 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget("FloorCounter", Label(self.__window, text="Floor\n"+str(self.__current_floor_num+1)),
                          77 / 128, 3 / 42, 9 / 128, 3 / 42, "TkDefaultFont %s", 12)
        self.__add_widget("GoUpFloorButton", Button(self.__window, text="Go Up\nFloor", command=lambda:
                                                    self.change_floor(1), bg="light grey"),
                          43 / 64, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget("HelpButton", Button(self.__window, text="Help", command=self.help, bg="light grey"),
                          25 / 32, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        if self.__current_stage == self.MAP_CREATION:
            return_text = "Return\nto Main\nMenu"
        elif self.__current_stage == self.NAV_GRAPH_TWEAKING:
            return_text = "Return\nto Map\nCreation"
        else:
            # Pathfinding stage
            return_text = "Return\nto Graph\nEditing"
        if self.__current_stage == self.PATHFINDING:
            confirm_text = "Return\nto Main\nMenu"
        else:
            # Pathfinding stage
            confirm_text = "Confirm"
        self.__add_widget("ReturnButton", Button(self.__window, text=return_text, command=self.prev_stage,
                                                 bg="light grey"),
                          111 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)
        self.__add_widget("ConfirmButton", Button(self.__window, text=confirm_text, command=self.next_stage,
                                                  bg="light grey"),
                          122 / 128, 3 / 42, 5 / 64, 5 / 42, "TkDefaultFont %s", 12)

    def prev_stage(self):
        self.__current_stage -= 1
        # Resets floor to floor 0
        self.__current_floor_num = 0
        self.__destroy_widgets()
        self.__active_tool = self.NONE
        if self.__current_stage == self.MAIN_MENU:
            self.__canvas.destroy()
            self.__canvas = None
            self.main_menu_ui()
        elif self.__current_stage == self.MAP_CREATION:
            self.map_creation_ui()
        elif self.__current_stage == self.NAV_MESH_GENERATION:
            self.prev_stage()
            return
        elif self.__current_stage == self.NAV_GRAPH_TWEAKING:
            self.__map.reset_nav_meshes()
            self.nav_graph_tweaking_ui()
        elif self.__current_stage == self.PATHFINDING:
            self.pathfinding_ui()
        if self.__current_stage > self.MAIN_MENU:
            self.update_canvas()
            if self.__current_stage != self.NAV_MESH_GENERATION:
                self.__place_standard_buttons()

    def next_stage(self):
        self.__current_stage += 1
        # Resets floor to floor 0
        self.__current_floor_num = 0
        self.__destroy_widgets()
        self.__active_tool = self.NONE
        if self.__current_stage >= self.RETURN:
            self.__current_stage = self.MAIN_MENU
            self.__canvas.destroy()
            self.__canvas = None
            self.main_menu_ui()
        elif self.__current_stage == self.MAP_CREATION:
            self.map_creation_ui()
            # Create canvas
            self.__canvas = Canvas(self.__window)
            self.__canvas.bind("<Button-1>", self.canvas_click)
            self.__canvas.place(anchor="center", relx=0.5, rely=4 / 7)
        elif self.__current_stage == self.NAV_MESH_GENERATION:
            self.nav_mesh_generation_ui()
            self.__cancel = False
            self.__map.generate_nav_graphs(self, self.__options[self.NODES_PER_EDGE])
            return
        elif self.__current_stage == self.NAV_GRAPH_TWEAKING:
            self.nav_graph_tweaking_ui()
        elif self.__current_stage == self.PATHFINDING:
            self.__map.join_links(self.__options[self.PRECOMPUTE_LINK_DIST],
                                  self.__options[self.PRIMARY_ALGORITHM],
                                  self.__options[self.LINK_WEIGHT]/10)
            self.pathfinding_ui()
            self.__start_pos = None
            self.__goal_pos = None
            self.__start_floor_num = 0
            self.__goal_floor_num = 0
            self.__path = None
        if self.__current_stage > self.MAIN_MENU:
            self.update_canvas()
            if self.__current_stage != self.NAV_MESH_GENERATION:
                self.__place_standard_buttons()

    def cancel(self):
        self.__cancel = True

    def key_down(self, event):
        if event.keycode == 16:
            # Program must store whether shift is currently held to alter wall placement behaviour
            self.__shift_held = True
        elif self.__current_stage not in (self.MAIN_MENU, self.NAV_MESH_GENERATION):
            if event.keycode == 49:
                # Number keys can be used instead of clicking buttons - 49 is number 1
                self.save()
            elif event.keycode == 50:
                self.open_options()
            elif event.keycode == 51:
                if self.__current_stage == self.MAP_CREATION:
                    self.__set_tool(self.CREATE_WALL)
                elif self.__current_stage == self.NAV_GRAPH_TWEAKING:
                    self.__set_tool(self.EDIT_WEIGHT)
                elif self.__current_stage == self.PATHFINDING:
                    self.__set_tool(self.PLACE_START)
            elif event.keycode == 52:
                if self.__current_stage == self.MAP_CREATION:
                    self.__set_tool(self.CREATE_LINK)
                elif self.__current_stage == self.NAV_GRAPH_TWEAKING:
                    self.__set_tool(self.CHANGE_DIRECTION)
                elif self.__current_stage == self.PATHFINDING:
                    self.__set_tool(self.PLACE_GOAL)
            elif event.keycode == 53:
                if self.__current_stage == self.MAP_CREATION:
                    self.__set_tool(self.DELETE)
                elif self.__current_stage == self.NAV_GRAPH_TWEAKING:
                    self.__set_tool(self.BLOCK_PATH)
                elif self.__current_stage == self.PATHFINDING:
                    self.__set_tool(self.FIND_PATH)
            elif event.keycode == 54:
                self.change_floor(-1)
            elif event.keycode == 55:
                self.change_floor(1)
            elif event.keycode == 56:
                self.help()
            elif event.keycode == 57:
                self.prev_stage()
            elif event.keycode == 48:
                # 0 is the furthest to the right on the keyboard, but has a lower key code
                self.next_stage()
            elif event.keycode == 18:
                # Alt key - generally in apps pressing alt should show what keys to press for hot keys, here it will
                # display a pop-up to explaining the 1-0 keys can be used.
                self.info("You can use the number keys (1-0) as shortcut buttons to activate any of the buttons at the "
                          "top of the screen.")

    def key_up(self, event):
        if event.keycode == 16:
            self.__shift_held = False
            self.shift_key_up()

    def shift_key_up(self):
        if self.__active_tool == self.FINISH_WALL:
            self.__active_tool = self.CREATE_WALL
            self.update_canvas()

    def update_canvas(self):
        self.__canvas_widget_data = []
        if self.__current_stage >= self.MAP_CREATION:
            if self.__options[self.ENABLE_GRID] and self.__current_stage == self.MAP_CREATION:
                for x in range(self.__options[self.GRID_SIZE_X]):
                    for y in range(self.__options[self.GRID_SIZE_Y]):
                        self.add_canvas_circle(Vec2D.bi_lerp(Vec2D(0, 0), Vec2D(GlobalConstants.FLOOR_RATIO.x, 0),
                                               Vec2D(0, GlobalConstants.FLOOR_RATIO.y),
                                               Vec2D(GlobalConstants.FLOOR_RATIO.x,
                                               GlobalConstants.FLOOR_RATIO.y),
                                               x/(self.__options[self.GRID_SIZE_X] - 1),
                                               y/(self.__options[self.GRID_SIZE_Y] - 1)), fill_colour="black",
                                               radius=0.01)
            vertex_positions = self.__current_floor.walls.vertex_positions
            for vert1_id in range(self.__current_floor.walls.vertices):
                for vert2_id in range(vert1_id, self.__current_floor.walls.vertices):
                    if self.__current_stage == self.MAP_CREATION:
                        if self.__current_floor.walls.get_edge_val(vert1_id, vert2_id) == 1:
                            self.add_canvas_line(vertex_positions[vert1_id], vertex_positions[vert2_id])
                        elif self.__current_floor.walls.get_edge_val(vert1_id, vert2_id) == 2:
                            self.add_canvas_line(vertex_positions[vert1_id], vertex_positions[vert2_id], width=0.004)
                    else:
                        if self.__current_floor.walls.get_edge(vert1_id, vert2_id):
                            self.add_canvas_line(vertex_positions[vert1_id], vertex_positions[vert2_id], width=0.004)
                if self.__current_stage == self.MAP_CREATION:
                    self.add_canvas_circle(vertex_positions[vert1_id], fill_colour="white")
            for link in self.__current_floor.links:
                self.add_canvas_circle(link.position, fill_colour="blue")
                self.add_canvas_text(Vec2D.sub(link.position, Vec2D(0, 0.04)), link.link_id)
        if self.__current_stage == self.NAV_GRAPH_TWEAKING:
            vertex_positions = self.__current_nav_mesh.nav_graph.vertex_positions
            for vert1_id in range(self.__current_nav_mesh.nav_graph.vertices):
                for vert2_id in range(vert1_id):
                    # Still need to display -1 edges
                    if self.__current_nav_mesh.nav_graph.get_edge_val(vert1_id, vert2_id) != 0:
                        weight1 = self.__current_nav_mesh.nav_graph.get_edge_val(vert1_id, vert2_id)
                        weight2 = self.__current_nav_mesh.nav_graph.get_edge_val(vert2_id, vert1_id)
                        if weight1 != -1:
                            if weight2 != -1:
                                # Edge is bi-directional
                                self.add_canvas_line(vertex_positions[vert1_id], vertex_positions[vert2_id],
                                                     dash=(2,), arrow=BOTH)
                            else:
                                # Edge is uni-directional from vert 1 to vert 2
                                self.add_canvas_line(vertex_positions[vert1_id], vertex_positions[vert2_id],
                                                     dash=(2,), arrow=FIRST)
                            display_weight = str(round(weight1 * 10, 2))
                            text_pos = Vec2D.add(vertex_positions[vert1_id],
                                                 vertex_positions[vert2_id]).scalar_multiply(1 / 2)
                            self.add_canvas_text(text_pos, display_weight)
                        else:
                            if weight2 != -1:
                                # Edge is uni-directional from vert 2 to vert 1
                                self.add_canvas_line(vertex_positions[vert1_id], vertex_positions[vert2_id],
                                                     dash=(2,), arrow=LAST)
                                display_weight = str(round(weight2 * 10, 2))
                            else:
                                # Edge is blocked
                                self.add_canvas_line(vertex_positions[vert1_id], vertex_positions[vert2_id],
                                                     dash=(2,))
                                display_weight = "∞"
                            text_pos = Vec2D.add(vertex_positions[vert1_id],
                                                 vertex_positions[vert2_id]).scalar_multiply(1 / 2)
                            self.add_canvas_text(text_pos, display_weight)
        elif self.__current_stage == self.PATHFINDING:
            if self.__start_floor_num == self.__current_floor_num:
                if self.__start_pos is not None:
                    self.add_canvas_circle(self.__start_pos, fill_colour="green")
            if self.__goal_floor_num == self.__current_floor_num:
                if self.__goal_pos is not None:
                    self.add_canvas_circle(self.__goal_pos, fill_colour="red")
            if self.__path is not None:
                vertex_positions = self.__current_nav_mesh.nav_graph.vertex_positions
                for edge_floor_tuple in self.__path.keys():
                    floor_num = edge_floor_tuple[1]
                    if floor_num == self.__current_floor_num:
                        path_to_display = self.__path[edge_floor_tuple]
                        if self.__options[self.SMOOTH_FINAL_PATH]:
                            control_verts = []
                            for vert_id in path_to_display:
                                control_verts.append(vertex_positions[vert_id])
                            self.__draw_cubic_spline(control_verts)
                        else:
                            for path_num in range(len(path_to_display) - 1):
                                vert1_id = path_to_display[path_num]
                                vert2_id = path_to_display[path_num + 1]
                                self.add_canvas_line(vertex_positions[vert1_id], vertex_positions[vert2_id],
                                                     dash=(2,), arrow=LAST)
        if self.__active_tool == self.FINISH_WALL:
            # Draws a circle at first wall vertex to make placement easier to judge
            if self.__options[self.ENABLE_GRID]:
                self.add_canvas_circle(self.__prev_click_pos, border_width=2, fill_colour="white")
            else:
                self.add_canvas_circle(self.__prev_click_pos, border_width=2)
        self.refresh_canvas()

    @staticmethod
    def __solve_coefficients(values):
        cubic_coefficients = list(None for _ in range(len(values) - 1))
        terms = len(values) * 4 - 4
        coefficients = []
        results = []
        for vert_num in range(0, len(values) - 1):
            coefficient_index = vert_num * 4
            # Require F{n}(x{n}) = y{n}
            coefficients.append(list(list(0 for _ in range(coefficient_index)) + [vert_num ** 3,
                                                                                  vert_num ** 2,
                                                                                  vert_num, 1] +
                                     list(0 for _ in range(terms - coefficient_index - 4))))
            results.append(values[vert_num])
            # Require F{n}(x{n+1}) = y{n+1}
            coefficients.append(list(list(0 for _ in range(coefficient_index)) + [(vert_num + 1) ** 3,
                                                                                  (vert_num + 1) ** 2,
                                                                                  (vert_num + 1), 1] +
                                     list(0 for _ in range(terms - coefficient_index - 4))))
            results.append(values[vert_num + 1])
            if vert_num != len(values) - 2:
                # Require F{n}'(x{n+1}) = F{n+1}'(x{n+1})
                coefficients.append(list(list(0 for _ in range(coefficient_index))
                                         + [3 * (vert_num + 1) ** 2,
                                            2 * (vert_num + 1),
                                            1, 0]
                                         + [-3 * (vert_num + 1) ** 2,
                                            -2 * (vert_num + 1),
                                            -1, 0] +
                                         list(0 for _ in range(terms - coefficient_index - 8))))
                results.append(0)
                # Require F{n}''(x{n+1}) = F{n+1}''(x{n+1})
                coefficients.append(list(list(0 for _ in range(coefficient_index))
                                         + [6 * (vert_num + 1),
                                            2, 0, 0]
                                         + [-6 * (vert_num + 1),
                                            -2, 0, 0] +
                                         list(0 for _ in range(terms - coefficient_index - 8))))
                results.append(0)
        # Require starting second derivative = 0
        coefficients.append(list([0, 2, 0, 0] + list(0 for _ in range(terms - 4))))
        results.append(0)
        # Require ending second derivative = 0
        coefficients.append(list(list(0 for _ in range(terms - 4)) + [6 * (len(values)-1), 2, 0, 0]))
        results.append(0)
        # Solve equations by finding the inverse matrix (could implement my own matrix and the Gauss-Jordan elimination
        # algorithm but I am running out of time so NumPy to the rescue!)
        coefficients_array = array(coefficients)
        results_array = array(results)
        inverse_coefficients = linalg.inv(coefficients_array)
        solutions = dot(inverse_coefficients, results_array)
        for solution_num in range(0, solutions.size, 4):
            cubic_coefficients[solution_num // 4] = [solutions[solution_num],
                                                     solutions[solution_num + 1],
                                                     solutions[solution_num + 2],
                                                     solutions[solution_num + 3]]
        return cubic_coefficients

    def __draw_cubic_spline(self, control_verts, accuracy=10):
        x_coefficients = self.__solve_coefficients(list(map(lambda vert: vert.x, control_verts)))
        y_coefficients = self.__solve_coefficients(list(map(lambda vert: vert.y, control_verts)))
        to_draw = []
        for cubic_num in range(len(control_verts)-1):
            for interval in range(0, accuracy):
                walls_copy = deepcopy(self.__current_floor.walls)
                start_t = cubic_num + (interval / accuracy)
                end_t = cubic_num + ((interval+1) / accuracy)
                start_pos = Vec2D(self.__evaluate_cubic(start_t, x_coefficients[cubic_num]),
                                  self.__evaluate_cubic(start_t, y_coefficients[cubic_num]))
                end_pos = Vec2D(self.__evaluate_cubic(end_t, x_coefficients[cubic_num]),
                                self.__evaluate_cubic(end_t, y_coefficients[cubic_num]))
                walls_copy.add_vertex(start_pos)
                walls_copy.add_vertex(end_pos)
                if Floor.check_for_intersections(walls_copy, walls_copy.vertices-1, walls_copy.vertices-2,
                                                 skip_case_1=True) and accuracy < 20:
                    control_verts = control_verts[0:cubic_num+1] + [
                                     Vec2D.add(Vec2D(self.__evaluate_cubic(cubic_num, x_coefficients[cubic_num]),
                                                     self.__evaluate_cubic(cubic_num, y_coefficients[cubic_num])),
                                               Vec2D(self.__evaluate_cubic(cubic_num+1, x_coefficients[cubic_num]),
                                                     self.__evaluate_cubic(cubic_num+1, y_coefficients[cubic_num]))
                                               ).scalar_multiply(0.5)] + control_verts[cubic_num+1::]
                    self.__draw_cubic_spline(control_verts, accuracy=accuracy+1)
                    return
                to_draw.append((start_pos, end_pos))
        for each_num in range(len(to_draw)):
            if each_num % accuracy == 0:
                self.add_canvas_line(to_draw[each_num][0], to_draw[each_num][1], dash=(2,), arrow=LAST)
            else:
                self.add_canvas_line(to_draw[each_num][0], to_draw[each_num][1], dash=(2,))

    @staticmethod
    def __evaluate_cubic(x, coefficients):
        return coefficients[0] * x ** 3 + coefficients[1] * x ** 2 + coefficients[2] * x + coefficients[3]

    def refresh_canvas(self):
        self.__canvas.delete("all")
        self.__canvas.create_line(0, 0, self.resolution.x, 0, width=5)
        for each in self.__canvas_widget_data:
            each.draw_scaled(self, self.__canvas)

    def canvas_click(self, event):
        click_pos = self.map_to_normalised(Vec2D(event.x, event.y))
        if (self.__options[self.ENABLE_GRID] and (self.__active_tool == self.CREATE_WALL or
                                                  self.__active_tool == self.FINISH_WALL)):
            # Round position to nearest grid point
            click_pos = Vec2D(round(click_pos.x * ((self.__options[self.GRID_SIZE_X]-1) /
                              GlobalConstants.FLOOR_RATIO.x)) * GlobalConstants.FLOOR_RATIO.x /
                              (self.__options[self.GRID_SIZE_X]-1),
                              round(click_pos.y * (self.__options[self.GRID_SIZE_Y]-1) /
                                    GlobalConstants.FLOOR_RATIO.y) * GlobalConstants.FLOOR_RATIO.y /
                              (self.__options[self.GRID_SIZE_Y]-1))
        if 0 <= click_pos.x <= GlobalConstants.FLOOR_RATIO.x and 0 <= click_pos.y <= GlobalConstants.FLOOR_RATIO.y:
            if self.__active_tool == self.CREATE_WALL:
                self.__prev_click_pos = click_pos
                self.__active_tool = self.FINISH_WALL
            elif self.__active_tool == self.FINISH_WALL:
                if not self.__current_floor.add_wall(self.__prev_click_pos, click_pos):
                    self.info("Walls and their vertices cannot intersect with other walls or themselves.")
                if self.__shift_held:
                    # If shift is held, continue placing from previous position
                    self.__prev_click_pos = click_pos
                else:
                    self.__active_tool = self.CREATE_WALL
            elif self.__active_tool == self.CREATE_LINK:
                # Create link window
                link_id = self.get_string("Enter link ID: (links with identical IDs are considered as joined together)."
                                          )
                if link_id is not None:
                    self.__current_floor.add_link(link_id, click_pos)
            elif self.__active_tool == self.DELETE:
                self.__current_floor.delete(click_pos)
            elif self.__active_tool == self.EDIT_WEIGHT:
                self.__current_nav_mesh.edit_weight(self, click_pos)
            elif self.__active_tool == self.CHANGE_DIRECTION:
                self.__current_nav_mesh.change_direction(self, click_pos)
            elif self.__active_tool == self.BLOCK_PATH:
                self.__current_nav_mesh.block_path(self, click_pos)
            elif self.__active_tool == self.PLACE_START:
                self.__start_pos = click_pos
                self.__map.reset_nav_meshes()
                self.__path = None
                self.__start_floor_num = self.__current_floor_num
            elif self.__active_tool == self.PLACE_GOAL:
                self.__goal_pos = click_pos
                self.__map.reset_nav_meshes()
                self.__path = None
                self.__goal_floor_num = self.__current_floor_num
            self.update_canvas()
        # Otherwise click is not inside bounds of interactable area of canvas

    def find_path(self):
        if self.__start_pos is not None and self.__goal_pos is not None:
            self.__map.reset_nav_meshes()
            self.__path = None
            self.__path = self.__map.pathfind(self.__start_pos, self.__start_floor_num, self.__goal_pos,
                                              self.__goal_floor_num, self.__options[self.PRIMARY_ALGORITHM],
                                              self.__options[self.LINK_ALGORITHM], self.__options[self.LINK_WEIGHT]/10,
                                              self.__options[self.LINK_WEIGHT]/10)
            if self.__path is None:
                self.info("No valid path between start and goal nodes.")
                self.__map.reset_nav_meshes()
            self.update_canvas()
        else:
            self.info("Must first select a start and goal before finding a path.")

    def map_to_canvas_space(self, point):
        # Converts from a vector position with components ranging between bounds of CANVAS_SCALE_RESOLUTION to a canvas
        # space pixel coordinate
        scaled_pos = point.scalar_multiply(self.min_canvas_scale * self.CANVAS_SCALE)
        offset = Vec2D.sub(Vec2D(self.resolution.x, self.resolution.y * 6 / 7),
                           GlobalConstants.FLOOR_RATIO.scalar_multiply(self.min_canvas_scale * self.CANVAS_SCALE)
                           ).scalar_multiply(1 / 2)
        return Vec2D.add(scaled_pos, offset)

    def map_to_normalised(self, point):
        # Converts from a canvas space pixel coordinate to a position vector with components ranging between bounds of
        # CANVAS_SCALE_RESOLUTION
        offset = Vec2D.sub(Vec2D(self.resolution.x, self.resolution.y * 6 / 7),
                           GlobalConstants.FLOOR_RATIO.scalar_multiply(self.min_canvas_scale * self.CANVAS_SCALE)
                           ).scalar_multiply(1 / 2)
        offset_pos = Vec2D.sub(point, offset)
        return offset_pos.scalar_multiply(1 / (self.min_canvas_scale * self.CANVAS_SCALE))

    def open_options(self):
        if self.__new_options is not None:
            OptionsManager(self, self.__new_options)
        else:
            OptionsManager(self, self.__options)

    def update_options(self, new_options):
        if self.__current_stage > Manager.MAP_CREATION:
            self.info("Setting changes made after the Map Creation stage will not be applied until you return to "
                      "the Main Menu or the Map Creation stage.")
            self.__new_options = new_options
        else:
            self.__options = new_options
            if self.__canvas is not None:
                self.update_canvas()

    def save(self):
        if self.__path is not None:
            # Navigation meshes must be properly reset before they are stored
            self.__map.reset_nav_meshes()
            self.__path = None
            self.update_canvas()
        data, data2 = self.__map.get_save_data(store_nav_graphs=self.__current_stage > self.MAP_CREATION)
        file1_obj = self.save_file(data)
        if data2 is not None and file1_obj is not None:
            # Path excluding map extension
            file2_obj = open(file1_obj.name[:-3] + "nav", mode="w")
            file2_obj.write(data2)
            file2_obj.close()
        if file1_obj is not None:
            file1_obj.close()

    def help(self):
        # Display help info
        if self.__current_stage == self.MAP_CREATION:
            self.info2(("You are currently on the map creation stage.\n\n"
                        "Walls can be created by clicking in two locations. If you want a wall to be connected to "
                        "another, simply click near the wall you want it to be joined to and it will be connected.\n"
                        "Walls cannot intersect other walls.\n"
                        "The shift key can be held down to place multiple walls at once.\n\n"
                        "Link nodes link between different floors, and can be used in place of staircases or elevators,"
                        " for example.\n"
                        "Simply place a link node and type in an ID, and then make sure to type in the same ID for any "
                        "other link nodes other floors (or even the same floor) that you want connected to it.\n\n"
                        "If you wish to delete a wall or link, simply select the delete tool and click the object you "
                        "want to be removed.\n\n"
                        "On all stages, you are able to navigate between floors by clicking on the 'go up floor' and "
                        "'go down floor' buttons.\n"
                        "During map creation, clicking these buttons create new floors; however, you will need to have "
                        "made at least one change to the current floor before creating a new one.\n\n"
                        "Settings:\n\n"
                        "'Enable grid' toggles on or off the grid that walls can be snapped to.\n"
                        "'Grid Size X & Y' modify the spacing of points on that said grid."))
        elif self.__current_stage == self.NAV_GRAPH_TWEAKING:
            self.info2(("You are currently on the nav graph tweaking stage.\n\n"
                        "The nav graph represents the straight line paths that can be potentially taken "
                        "through the map\n\n"
                        "The weights of the edges of this graph are initialised based on distance, but can be changed "
                        "by selecting the 'edit weight' tool and clicking on the edge you want to edit the weight of."
                        "\n\n"
                        "The change arrows on each of the edges represent the directions they can be travelled down."
                        "The 'change direction' tool can be used by clicking on these edges to cycle through"
                        "the edge being bi-directional, uni-directional one way and uni-directional the other.\n\n"
                        "The 'block path' tool can be used to prevent travel down that edge completely.\n\n"
                        "Settings:\n\n"
                        "'Link weight'determines the default weight edges between links sharing the same ID in the link"
                        "graph. I.E. the cost of travelling between two links."))
        elif self.__current_stage == self.PATHFINDING:
            self.info2(("You are currently on the path finding stage stage.\n\n"
                        "Here, you can place a start and end point using the 'place start' and 'place goal' tools."
                        "You can then find a path between the two points by clicking 'find path'.\n\n"
                        "Settings:\n\n"
                        "'Precomp Link Dist' determines whether paths between every pair of links on the same floor are"
                        " precomputed and stored or if they are just verified to be accessible and then a distance "
                        "heuristic is used instead.\n"
                        "'Smooth Path' determines whether the final path is smoothed using cubic spline interpolation."
                        "'Link Algorithm' determines the graph search algorithm used for pathfinding on the link graph "
                        "to find paths between floors."
                        ))

    def change_floor(self, change):
        if self.__current_floor_num + change in self.__map.floors.keys():
            self.__current_floor_num += change
        elif self.__current_stage == self.MAP_CREATION:
            if self.__current_floor.edited:
                self.__current_floor_num += change
                self.__map.floors[self.__current_floor_num] = Floor()
            # Otherwise, do not change floor
        # Otherwise, do not change floor as it was not created
        self.update_canvas()
        self.__widget_refs["FloorCounter"].configure(text="Floor\n"+str(self.__current_floor_num+1))

    def __set_tool(self, new_tool):
        self.__active_tool = new_tool
        self.update_canvas()
        # Makes selected tool appear pressed and raises others to make it obvious which tool is currently selected
        if self.__current_stage == self.MAP_CREATION:
            self.__widget_refs[self.CREATE_WALL].configure(relief=RAISED, bg="light grey")
            self.__widget_refs[self.CREATE_LINK].configure(relief=RAISED, bg="light grey")
            self.__widget_refs[self.DELETE].configure(relief=RAISED, bg="light grey")
        elif self.__current_stage == self.NAV_GRAPH_TWEAKING:
            self.__widget_refs[self.EDIT_WEIGHT].configure(relief=RAISED, bg="light grey")
            self.__widget_refs[self.CHANGE_DIRECTION].configure(relief=RAISED, bg="light grey")
            self.__widget_refs[self.BLOCK_PATH].configure(relief=RAISED, bg="light grey")
        elif self.__current_stage == self.PATHFINDING:
            self.__widget_refs[self.PLACE_START].configure(relief=RAISED, bg="light grey")
            self.__widget_refs[self.PLACE_GOAL].configure(relief=RAISED, bg="light grey")
            self.__widget_refs[self.FIND_PATH].configure(relief=RAISED, bg="light grey")
        self.__widget_refs[new_tool].configure(relief=SUNKEN, bg="#F0F0F0")

    def open_file(self):
        self.shift_key_up()
        file1_obj = filedialog.askopenfile(filetypes=[("Map file", "*.map")])
        if file1_obj is None:
            file2_obj = None
        else:
            try:
                file2_obj = open(file1_obj.name[:-3]+"nav")
            except IOError:
                file2_obj = None
        return file1_obj, file2_obj

    def save_file(self, data):
        self.shift_key_up()
        file_obj = filedialog.asksaveasfile(defaultextension=".map",
                                            filetypes=[("Map file", "*.map")],
                                            initialfile="Map")
        if file_obj is not None:
            file_obj.write(data)
        return file_obj

    def warning(self, text):
        if self.__options[self.DISABLE_WARNINGS]:
            return True
        else:
            # As key releases cannot be detected when the pop-up is open, shift must be released
            self.shift_key_up()
            return messagebox.askokcancel("WARNING", message=text +
                                          "\nPop-ups and warnings such as these can be disabled in the options menu.")

    def info(self, text):
        if not self.__options[self.DISABLE_WARNINGS]:
            # As key releases cannot be detected when the pop-up is open, shift must be released
            self.shift_key_up()
            return messagebox.showinfo("INFO", message=text +
                                       "\nPop-ups and warnings such as these can be disabled in the options menu.")

    def info2(self, text):
        # As key releases cannot be detected when the pop-up is open, shift must be released
        self.shift_key_up()
        return messagebox.showinfo("HELP", message=text)

    def get_float(self, text):
        # As key releases cannot be detected when the pop-up is open, shift must be released
        self.shift_key_up()
        return simpledialog.askfloat("INPUT", prompt=text)

    def get_string(self, text):
        # As key releases cannot be detected when the pop-up is open, shift must be released
        self.shift_key_up()
        return simpledialog.askstring("INPUT", prompt=text)


class OptionsManager:

    # Category constants
    GENERAL = "General"
    MAP_CREATION = "Map Creation"
    NAV_MESH_GENERATION = "Navmesh Generation"
    NAV_GRAPH_EDITING = "Navgraph Editing"
    PATHFINDING = "Pathfinding"

    def __init__(self, master, options):
        self.__window = Toplevel()
        self.__window.title("Options")
        self.__window.geometry("360x480")
        self.__window.resizable(0, 0)
        self.__option_widgets = []
        self.__master = master
        self.__options = options
        self.__vars = dict()
        # Never need to reference or delete
        Label(self.__window, text="Options", font="TkDefaultFont 14 bold").place(anchor="center", relx=0.5, rely=1/16,
                                                                                 relwidth=1/4, relheight=1/16)
        Separator(self.__window, orient=HORIZONTAL).place(anchor="center", relx=0.5, rely=7 / 32, relwidth=1,
                                                          relheight=0)
        Separator(self.__window, orient=VERTICAL).place(anchor="center", relx=1/3, rely=0.5, relwidth=0, relheight=9/16)
        Separator(self.__window, orient=HORIZONTAL).place(anchor="center", relx=0.5, rely=25 / 32, relwidth=1,
                                                          relheight=0)
        self.__category = StringVar(self.__window)
        self.__category.set(self.GENERAL)
        self.__category_drop_down = OptionMenu(self.__window, self.__category, self.GENERAL, self.MAP_CREATION,
                                               self.NAV_MESH_GENERATION, self.NAV_GRAPH_EDITING, self.PATHFINDING,
                                               command=self.__category_change)
        self.__category_drop_down.configure(font="TkDefaultFont 12")
        self.__category_drop_down.place(anchor="center", relx=0.5, rely=5/32, relwidth=7/12, relheight=1/16)
        self.__category_change(None)

        self.__apply_button = Button(self.__window, text="Apply", command=self.__apply, bg="light grey",
                                     font="TkDefaultFont 12")
        self.__apply_button.place(anchor="center", relx=41/48, rely=57/64, relwidth=5/24, relheight=5/32)
        self.__cancel_button = Button(self.__window, text="Cancel", command=self.__cancel, bg="light grey",
                                      font="TkDefaultFont 12")
        self.__cancel_button.place(anchor="center", relx=7 / 48, rely=57 / 64, relwidth=5 / 24, relheight=5 / 32)

        self.__window.mainloop()

    def __category_change(self, _):
        self.__clear_option_widgets()
        if self.__category.get() == self.GENERAL:
            self.__place_general_options()
        elif self.__category.get() == self.MAP_CREATION:
            self.__place_map_creation_options()
        elif self.__category.get() == self.NAV_MESH_GENERATION:
            self.__place_nav_mesh_generation_options()
        elif self.__category.get() == self.NAV_GRAPH_EDITING:
            self.__place_nav_graph_editing_options()
        elif self.__category.get() == self.PATHFINDING:
            self.__place_pathfinding_options()
        else:
            raise Exception("Invalid category selected.")
        self.__window.update()

    def __place_option_widget(self, option_name, widget, widget2=None):
        # skip is required for string options that will use the OptionMenu widget as OptionMenus cannot be instantiated
        # without a variable, unlike other tkinter widgets.
        skip = False
        if type(self.__options[option_name]) is bool:
            self.__vars[option_name] = BooleanVar()
        elif type(self.__options[option_name]) is int:
            self.__vars[option_name] = IntVar()
        elif type(self.__options[option_name]) is str:
            skip = True
        elif type(self.__options[option_name]) is float:
            self.__vars[option_name] = DoubleVar()
        else:
            raise Exception("Unsupported option type.")
        if not skip:
            self.__vars[option_name].set(self.__options[option_name])
            if widget2 is None:
                widget.configure(variable=self.__vars[option_name], command=lambda: self.__update_var(option_name))
            else:
                widget2.configure(variable=self.__vars[option_name], command=lambda _: self.__update_var(option_name))
                widget.configure(textvariable=self.__vars[option_name])
        widget_num = len(self.__option_widgets)//2
        label = Label(self.__window, text=option_name, font="TkDefaultFont 14")
        label.place(anchor="center", relx=1/6, rely=5/16+widget_num/8, relwidth=1/4, relheight=3/32)
        self.__option_widgets.append(label)
        if skip:
            widget.place(anchor="center", relx=2 / 3, rely=5 / 16 + widget_num / 8, relwidth=1 / 2, relheight=1 / 16)
        else:
            widget.place(anchor="center", relx=7/8, rely=5/16+widget_num/8, relwidth=1/12, relheight=1/16)
        self.__option_widgets.append(widget)
        if widget2 is not None:
            widget2.place(anchor="center", relx=29 / 48, rely=5 / 16 + widget_num / 8, relwidth=3 / 8, relheight=1 / 24)
            self.__option_widgets.append(widget2)

    def __place_general_options(self):
        self.__place_option_widget(Manager.DISABLE_WARNINGS, Checkbutton(self.__window))
        # Currently multi-threading is not implemented so thread target does not need to be an available option
        # self.__place_option_widget(Manager.THREAD_TARGET, Label(self.__window,
        # font="TkDefaultFont 14", relief="solid", borderwidth=1),
        #                           Scale(self.__window, resolution=1, from_=2, to=28, orient=HORIZONTAL, showvalue=0))
        # OptionMenu widget cannot be initialised without assigning a variable so the variable must be created
        # here instead.
        self.__vars[Manager.PRIMARY_ALGORITHM] = StringVar()
        self.__vars[Manager.PRIMARY_ALGORITHM].set(self.__options[Manager.PRIMARY_ALGORITHM])
        self.__place_option_widget(Manager.PRIMARY_ALGORITHM, OptionMenu(self.__window, self.__vars[
            Manager.PRIMARY_ALGORITHM], GlobalConstants.A_STAR, GlobalConstants.DIJKSTRA, GlobalConstants.GREEDY,
                                   command=lambda _: self.__update_var(Manager.PRIMARY_ALGORITHM)))

    def __place_map_creation_options(self):
        self.__place_option_widget(Manager.ENABLE_GRID, Checkbutton(self.__window))
        self.__place_option_widget(Manager.GRID_SIZE_X, Label(self.__window,
                                                              font="TkDefaultFont 14", relief="solid", borderwidth=1),
                                   Scale(self.__window, resolution=1, from_=2, to=29, orient=HORIZONTAL, showvalue=0))
        self.__place_option_widget(Manager.GRID_SIZE_Y, Label(self.__window,
                                                              font="TkDefaultFont 14", relief="solid", borderwidth=1),
                                   Scale(self.__window, resolution=1, from_=2, to=17, orient=HORIZONTAL, showvalue=0))

    def __place_nav_mesh_generation_options(self):
        self.__place_option_widget(Manager.NODES_PER_EDGE, Label(self.__window, font="TkDefaultFont 14", relief="solid",
                                                                 borderwidth=1),
                                   Scale(self.__window, resolution=1, from_=1, to=3, orient=HORIZONTAL, showvalue=0))

    def __place_nav_graph_editing_options(self):
        self.__place_option_widget(Manager.LINK_WEIGHT, Label(self.__window, font="TkDefaultFont 12", relief="solid",
                                                              borderwidth=1),
                                   Scale(self.__window, resolution=0.1, from_=0.1, to=20.0, orient=HORIZONTAL,
                                         showvalue=0))

    def __place_pathfinding_options(self):
        self.__place_option_widget(Manager.PRECOMPUTE_LINK_DIST, Checkbutton(self.__window))
        self.__place_option_widget(Manager.SMOOTH_FINAL_PATH, Checkbutton(self.__window))
        # OptionMenu widget cannot be initialised without assigning a variable so the variable must be created
        # here instead.
        self.__vars[Manager.LINK_ALGORITHM] = StringVar()
        self.__vars[Manager.LINK_ALGORITHM].set(self.__options[Manager.LINK_ALGORITHM])
        self.__place_option_widget(Manager.LINK_ALGORITHM, OptionMenu(self.__window, self.__vars[
            Manager.LINK_ALGORITHM], GlobalConstants.DIJKSTRA, GlobalConstants.BFS, command=lambda _: self.__update_var(
                                                                             Manager.LINK_ALGORITHM)))

    def __update_var(self, key):
        self.__options[key] = self.__vars[key].get()

    def __clear_option_widgets(self):
        for option_widget in self.__option_widgets:
            option_widget.destroy()
        self.__option_widgets = []
        self.__vars = dict()

    def __apply(self):
        self.__master.update_options(deepcopy(self.__options))

    def __cancel(self):
        self.__window.destroy()
