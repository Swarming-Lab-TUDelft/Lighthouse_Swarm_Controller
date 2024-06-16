import tkinter as tk
from tkinter import ttk
import ctypes as ct

dark1 = "#17181a"
dark2 = "#202123"
dark3 = "#202123"

text_white = "#f7f7f7"
text_green = "#379c7c"
text_red = "#c05646"
text_grey = "#494a4c"
text_light_grey = "#d0d0d0"

grey_line = "#333436"
bar_grey = "#494a4c"
bar_green = "#02b075"
bar_red = "#e61102"
card_hover = "#333436"

button_colors = {
    "green": ("#02B075", "#029665", "#02E398", "#016343"),
    "red": ("#E61102", "#CC1002", "#EA5348", "#990C02"),
    "grey": ("#494A4C", "#3D3E40", "#5A5A63", "#313233"),
    "purple": ("#702A8C", "#5E2679", "#8A51B1", "#4B1A64"),
    "blue": ("#0B85FF", "#0A71D8", "#3E9CEB", "#0963AB"),
    "yellow": ("#FFC107", "#D4A006", "#FFD453", "#B78B05"),
    "orange": ("#FF6D00", "#DD5F00", "#FF8A3B", "#AC4E00"),
    "green2": ("#4CAF50", "#43A047", "#66BB6A", "#388E3C")
}

class window(tk.Tk):
    def __init__(self, name):
        super().__init__()
        self.title(name)
        self.resizable(True, True)
        self.configure(cursor="left_ptr", bg='black')

        style = ttk.Style()
        style.configure('Front.TFrame', background=dark2)
        style.configure('Back.TFrame', background=dark1)
        style.configure('Card.TFrame', background=dark3)
        style.configure('TLabel', font=('Roboto', 11), foreground=text_white, background=dark2)
        style.configure('grey.TLabel', font=('Roboto', 11), foreground=text_grey, background=dark2)
        style.configure('Header.TLabel', font=('Roboto', 11, 'bold'), foreground=text_white, background=dark1)
        style.configure('TButton', font=('Roboto', 10), foreground=text_white, background=dark2)
        style.configure('green.TButton', font=('Roboto', 10), foreground=text_white, background=bar_green)
        style.configure('red.TButton', font=('Roboto', 10), foreground=text_white, background="#ff463a")
        style.configure('separator.TFrame', background=grey_line)
        style.configure('bar_grey.TFrame', background=bar_grey)
        style.configure('bar_green.TFrame', background=bar_green)


class LabelFrame(ttk.Frame):
    def __init__(self, root, text, padding=8, scroll=False):
        self.outer_frame = ttk.Frame(root, padding=8, style="Back.TFrame")
        ttk.Label(self.outer_frame, text=text.upper(), style="Header.TLabel", padding="2pt").pack(side="top", fill="x")

        if scroll:
            self.canvas = tk.Canvas(self.outer_frame, highlightthickness=0, background=dark2, width=216)
            super().__init__(self.canvas, padding=padding, style="Front.TFrame")
            self.canvas.pack(side="top", fill="both", expand=True)

            self.bind(
                "<Configure>",
                lambda e: self.canvas.configure(
                    scrollregion=self.canvas.bbox("all"),
                    width=e.width
                )
            )

            self.scroll_bar = ttk.Scrollbar(self.outer_frame, orient="vertical", command=self.canvas.yview)
            # self.scroll_bar.pack(side="left", fill="y")
            self.canvas.create_window((0, 0), window=self, anchor="nw")
            self.canvas.configure(yscrollcommand=self.scroll_bar.set)

            
            self.canvas.bind('<Enter>', self._bound_to_mousewheel)
            self.canvas.bind('<Leave>', self._unbound_to_mousewheel)
        else:
            super().__init__(self.outer_frame, padding=padding, style="Front.TFrame")
            super().pack(side="top", fill="both", expand=True)


    def pack(self, *args, **kwargs):
        self.outer_frame.pack(*args, **kwargs)
    
    def grid(self, *args, **kwargs):
        self.outer_frame.grid(*args, **kwargs)
    
    def _bound_to_mousewheel(self, event):
        self.canvas.bind_all("<Button-4>", self._scroll_up)
        self.canvas.bind_all("<Button-5>", self._scroll_down)

    def _unbound_to_mousewheel(self, event):
        self.canvas.unbind_all("<Button-4>")
        self.canvas.unbind_all("<Button-5>")
    
    def _scroll_up(self, event):
        self.canvas.yview_scroll(-1, "units")
    
    def _scroll_down(self, event):
        self.canvas.yview_scroll(1, "units")


class DroneCard(tk.Frame):
    def __init__(self, root, uri, state, progress, command):
        super().__init__(root, background=dark3, pady=8, padx=8)
        self.command = command
        self.active = False

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.id_label = ttk.Label(self, text=uri.split('E')[-1].lstrip('0'))
        self.id_label.grid(row=0, column=0, sticky="w")
        self.state_label = ttk.Label(self, text=state)
        self.state_label.grid(row=0, column=1, sticky="e")
        self.bar = Bar(self, 10, 200, progress)
        self.bar.grid(row=1, column=0, columnspan=2, pady="2pt")


        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)
        self.bind("<Button-1>", self.pressed)
        self.id_label.bind("<Button-1>", self.pressed)
        self.state_label.bind("<Button-1>", self.pressed)
        self.bar.bind("<Button-1>", self.pressed)
        self.bar.front_bar.bind("<Button-1>", self.pressed)
    
    def on_enter(self, event=None):
        self.configure(background=card_hover)
        self.state_label.configure(background=card_hover)
        self.id_label.configure(background=card_hover)
    
    def on_leave(self, event=None):
        if not self.active:
            self.configure(background=dark3)
            self.state_label.configure(background=dark3)
            self.id_label.configure(background=dark3)
    
    def pressed(self, event=None):
        self.command()


class RadioCard(tk.Frame):
    def __init__(self, root, radio, state, command):
        super().__init__(root, background=dark3, pady=8, padx=8)
        self.active = False
        self.command = command

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.id_label = ttk.Label(self, text=radio)
        self.id_label.grid(row=0, column=0, sticky="w")
        self.state_label = ttk.Label(self, text=state)
        self.state_label.grid(row=0, column=1, sticky="e")

        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)
        self.bind("<Button-1>", self.pressed)
        self.id_label.bind("<Button-1>", self.pressed)
        self.state_label.bind("<Button-1>", self.pressed)
    
    def on_enter(self, event=None):
        self.configure(background=card_hover)
        self.state_label.configure(background=card_hover)
        self.id_label.configure(background=card_hover)
    
    def on_leave(self, event=None):
        if not self.active:
            self.configure(background=dark3)
            self.state_label.configure(background=dark3)
            self.id_label.configure(background=dark3)
    
    def pressed(self, event=None):
        self.command()


class DroneDataFrame(tk.Frame):
    def __init__(self, root, uri, state, params, command_el=None, command_rl=None, indv_takeoff=None):
        super().__init__(root, background=dark2)

        self.command_el = command_el
        self.command_rl = command_rl
        self.indv_takeoff = indv_takeoff
        
        self.title_label = ttk.Label(self, text="Drone " + uri.split('E')[-1].lstrip('0'), background=dark2, anchor="center")
        self.title_label.pack(fill="x", side="top", pady=(0, 20))

        button_frame = ttk.Frame(self, style='Front.TFrame')
        button_frame.pack(fill="x", side="top", pady=(0, 20))
        self.emerg_button = RoundedButton(button_frame, text="Land in place", width=150, color="red", command=lambda uri_i=uri.split("/")[-1]: command_el(uri_i))
        self.emerg_button.pack(side="left", padx=2)
        self.indv_takeoff_button = RoundedButton(button_frame, text="Take off", width=150, color="yellow", command=lambda uri_i=uri.split('/')[-1]: indv_takeoff(uri_i))
        self.indv_takeoff_button.pack(side="top", padx=2)
        self.return_button = RoundedButton(button_frame, text="Return to landing pad", width=150, color="green", command=lambda uri_i=uri.split("/")[-1]: command_rl(uri_i))
        self.return_button.pack(side="right", padx=2)
        
        ########## DATA FRAME ##########
        data_frame = ttk.Frame(self, style='Front.TFrame')
        data_frame.pack(fill="x", side="top", pady=(0, 20))
        data_frame.columnconfigure(0, weight=1)
        data_frame.columnconfigure(1, weight=1)
        SeparatorH(data_frame).grid(row=0, column=0, columnspan=2, sticky="ew")

        ttk.Label(data_frame, text="ID:").grid(row=1, column=0, sticky="w")
        self.id = ttk.Label(data_frame, text=uri)
        self.id.grid(row=1, column=1, sticky="e")

        SeparatorH(data_frame).grid(row=2, column=0, columnspan=2, sticky="ew")

        ttk.Label(data_frame, text="Radio:").grid(row=3, column=0, sticky="w")
        self.radio = ttk.Label(data_frame, text=params["radio"])
        self.radio.grid(row=3, column=1, sticky="e")

        SeparatorH(data_frame).grid(row=4, column=0, columnspan=2, sticky="ew")

        ttk.Label(data_frame, text="State:").grid(row=5, column=0, sticky="w")
        self.state = ttk.Label(data_frame, text=state)
        self.state.grid(row=5, column=1, sticky="e")

        SeparatorH(data_frame).grid(row=6, column=0, columnspan=2, sticky="ew")

        ttk.Label(data_frame, text="Battery:").grid(row=7, column=0, sticky="w")
        battery_frame = ttk.Frame(data_frame, style='Front.TFrame')
        battery_frame.grid(row=7, column=1, sticky="e")
        self.battery_bar = Bar(battery_frame, 10, 120, float(params["bat_level"]))
        self.battery_bar.pack(side="right")
        self.battery_label = ttk.Label(battery_frame, text=str(round(float(params["bat_level"])*100, 2)) + "%")
        self.battery_label.pack(side="right", padx=(0, 5))

        SeparatorH(data_frame).grid(row=8, column=0, columnspan=2, sticky="ew")

        ttk.Label(data_frame, text="Position: (x,y,z) [m]").grid(row=9, column=0, sticky="w")
        self.position = ttk.Label(data_frame, text="(" + ", ".join([str(round(float(x), 3)) for x in params["pos"]]) + ")")
        self.position.grid(row=9, column=1, sticky="e")

        SeparatorH(data_frame).grid(row=10, column=0, columnspan=2, sticky="ew")

        ttk.Label(data_frame, text="Velocity: (x,y,z) [m]").grid(row=11, column=0, sticky="w")
        self.velocity = ttk.Label(data_frame, text="(" + ", ".join([str(round(float(x), 3)) for x in params["vel"]]) + ")")
        self.velocity.grid(row=11, column=1, sticky="e")

        SeparatorH(data_frame).grid(row=14, column=0, columnspan=2, sticky="ew")

        ########## TEXT BOX ##########
        self.text_box = TextBox(self, 10)
        self.text_box.pack(side='bottom', fill="x", expand=False)
    
    def update(self, uri, state, params, msgs):
        self.title_label.configure(text="Drone " + uri.split('E')[-1].lstrip('0'))
        self.id.configure(text=uri)
        self.state.configure(text=state)
        self.radio.configure(text=params["radio"])
        self.battery_bar.set_progress(float(params["bat_level"]))
        self.battery_label.configure(text=str(round(float(params["bat_level"]), 2)) + "%")
        self.position.configure(text="(" + ", ".join([str(round(float(x), 3)) for x in params["pos"]]) + ")")
        self.velocity.configure(text="(" + ", ".join([str(round(float(x), 3)) for x in params["vel"]]) + ")")
        self.text_box.update(msgs)
        self.emerg_button.command = lambda uri_i=uri.split("/")[-1]: self.command_el(uri_i)
        self.indv_takeoff_button.command = lambda uri_i=uri.split("/")[-1]: self.indv_takeoff(uri_i)
        self.return_button.command = lambda uri_i=uri.split("/")[-1]: self.command_rl(uri_i)


class RadioDataFrame(tk.Frame):
    def __init__(self, root, id, data):
        super().__init__(root, background=dark2)

        self.id = id
        self.data = data
        self.response_labels = []

        self.title_label = ttk.Label(self, text=f"Radio {id}", background=dark2, anchor="center")
        self.title_label.pack(fill="x", side="top", pady=(0, 20))

        self.data_frame = ttk.Frame(self, style='Front.TFrame')
        self.data_frame.pack(fill="x", side="top", pady=(0, 20))
        self.data_frame.columnconfigure(0, weight=0)
        self.data_frame.columnconfigure(1, weight=1, uniform="radio_data1")
        self.data_frame.columnconfigure(2, weight=0)
        self.data_frame.columnconfigure(3, weight=0)
        self.data_frame.columnconfigure(4, weight=1, uniform="radio_data1")
        SeparatorH(self.data_frame).grid(row=0, column=0, columnspan=5, sticky="ew")
        
        self._create_rows(1)

        ########## TEXT BOX ##########
        self.text_box = TextBox(self, 10)
        self.text_box.pack(side='bottom', fill="x", expand=False)
    
    def update(self, id, data, msgs):
        if id != self.id:
            self.id = id
            self.data = data
            self.title_label.configure(text=f"Radio {id}")
            
            for child in self.data_frame.winfo_children():
                child.destroy()
            
            SeparatorH(self.data_frame).grid(row=0, column=0, columnspan=5, sticky="ew")
            self._create_rows(1)
        else:
            self.data = data
            for i, label in enumerate(self.response_labels):
                label.configure(text=self.data["response"][i])
        
        self.text_box.update(msgs)


    def _create_rows(self, start):
        self.response_labels = []
        for i, uri in enumerate(self.data["uris"]):
            if i > len(self.data["response"])-1:
                self.data["response"].append("nothing published")
            col = i % 2
            row = (i // 2) * 2 + start
            ttk.Label(self.data_frame, text=uri.split('E')[-1].lstrip('0')).grid(row=row, column=col*3, sticky="w")
            self.response_labels.append(ttk.Label(self.data_frame, text=self.data["response"][i]))
            self.response_labels[-1].grid(row=row, column=col*3+1, sticky="e")
            SeparatorV(self.data_frame).grid(row=row, column=2, sticky="ns", padx=(10, 10))
            SeparatorH(self.data_frame).grid(row=row+1, column=0, columnspan=5, sticky="ew")


class SwarmDataFrame(tk.Frame):
    def __init__(self, root, data, commands):
        super().__init__(root, background=dark2)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)

        RoundedButton(self, text="Emergency land", width=200, color="red", command=commands["emergency_land"]).grid(row=0, column=0, sticky="w")
        RoundedButton(self, text="Return all", width=200, command=commands["return_all"]).grid(row=0, column=1, sticky="e")
        change_drone_count_frame = ttk.Frame(self, style='Front.TFrame')
        change_drone_count_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(20, 5))
        ttk.Label(change_drone_count_frame, text="Required number of swarming drones: ").pack(side="left")
        RoundedButton(change_drone_count_frame, text="+", width=30, command=commands["add_drone"]).pack(side="right")
        self.req_swarming = ttk.Label(change_drone_count_frame, text="0")
        self.req_swarming.pack(side="right", padx=(10, 10))
        RoundedButton(change_drone_count_frame, text="-", width=30, command=commands["remove_drone"]).pack(side="right")

        ttk.Label(self, text="Statistics: ").grid(row=2, column=0, columnspan=2, sticky="w", pady=(0, 5))

        self.stats_frame = ttk.Frame(self, style='Front.TFrame')
        self.stats_frame.grid(row=3, column=0, columnspan=2, sticky="nsew")
        self.stats_frame.columnconfigure(0, weight=0)
        self.stats_frame.columnconfigure(1, weight=1, uniform="swarm_data1")
        self.stats_frame.columnconfigure(2, weight=0)
        self.stats_frame.columnconfigure(3, weight=0)
        self.stats_frame.columnconfigure(4, weight=1, uniform="swarm_data1")
        SeparatorH(self.stats_frame).grid(row=0, column=0, columnspan=5, sticky="ew")

        data_labels = {
            "act_num_drones": "Flying: ",
            "num_drones_landing": "Landing: ",
            "num_drones_init": "Initialising: ",
            "num_drones_charging": "Charging: ",
            "num_drones_charged": "Charged: ",
            "num_drones_error": "Error: ",
        }
        self.stats_labels = {}
        for i, (stat, name) in enumerate(data_labels.items()):
            col = i % 2
            row = (i // 2) * 2 + 1
            ttk.Label(self.stats_frame, text=name).grid(row=row, column=col*3, sticky="w")
            self.stats_labels[stat] = ttk.Label(self.stats_frame, text=data[stat])
            self.stats_labels[stat].grid(row=row, column=col*3+1, sticky="e")
            SeparatorV(self.stats_frame).grid(row=row, column=2, sticky="ns", padx=(10, 10))
            SeparatorH(self.stats_frame).grid(row=row+1, column=0, columnspan=5, sticky="ew")

        row = 5
        for header, command_list in list(commands.items())[4:]:
            ttk.Label(self, text=header).grid(row=row, column=0, columnspan=2, sticky="w", pady=(10, 5))
            col = 0
            row += 1
            for name, command in command_list.items():
                RoundedButton(self, text=name, width=100, command=command).grid(row=row, column=col, pady=(1, 1))
                row += col
                col = 1 - col

    
    def update(self, data):
        for stat, label in self.stats_labels.items():
            label.configure(text=data[stat])
        self.req_swarming.configure(text=data["req_num_drones"])


class TextBox(ttk.Frame):
    def __init__(self, root, height, msgs=["",], label="Messages:"):
        super().__init__(root, style="Front.TFrame")
        self.msgs = msgs
        # self.pack_propagate(False)
        ttk.Label(self, text=label).pack(side='top', fill="x")
        self.text_box = tk.Text(self, height=height, font=("Liberation Mono", 11), background=dark1, foreground=text_white, insertbackground=text_white, borderwidth=0, highlightthickness=0, wrap="word", padx=2, pady=2)
        for msg in msgs:
            self.text_box.insert("end", msg + "\n")
        self.text_box.configure(state="disabled")
        self.text_box.see("end")
        self.text_box.pack(side='top', fill="both", expand=True)

    def update(self, msgs):
        if msgs != self.msgs:
            self.text_box.configure(state="normal")
            self.text_box.delete("1.0", "end")
            for msg in msgs:
                self.text_box.insert("end", msg + "\n")
            self.text_box.configure(state="disabled")
            self.text_box.see("end")
            self.msgs = msgs


class SeparatorH(ttk.Frame):
    def __init__(self, root):
        super().__init__(root, height=1, style="separator.TFrame")


class SeparatorV(ttk.Frame):
    def __init__(self, root):
        super().__init__(root, width=1, style="separator.TFrame")


class Bar(ttk.Frame):
    def __init__(self, root, height, width, progress):
        self.width = width
        super().__init__(root, style="bar_grey.TFrame", width=width, height=height)
        self.pack_propagate(False)
        self.front_bar = tk.Frame(self, width=width*progress/100, height=height, background=bar_green)
        self.front_bar.pack(side="left", fill="y")
    
    def set_progress(self, progress):
        if progress >= 0 and progress <= 100:
            self.front_bar.configure(width=self.width*progress/100, background=bar_green)
        elif progress > 100:
            self.front_bar.configure(width=self.width, background=bar_green)
        else:
            self.front_bar.configure(width=self.width, background=bar_red)


class Button(ttk.Button):
    def __init__(self, root, text, color, command=None):
        if color == "green":
            super().__init__(root, text=text, command=command, style="green.TButton", padding="2pt")
        elif color == "red":
            super().__init__(root, text=text, command=command, style="red.TButton", padding="2pt")


    def __init__(self, name, e_stop_cmd):
        super().__init__()
        self.title(name)
        self.resizable(True, True)
        self.configure(cursor="left_ptr", bg='black')

        # Create a frame for the E-STOP button
        estop_frame = ttk.Frame(self, style="Back.TFrame", padding=8)
        estop_frame.pack(side="bottom", fill="x")


class EStopFrame(tk.Frame):
    def __init__(self, root, e_stop_cmd):
        super().__init__(root, background=dark2)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)

        # Create the E-STOP button with a red background
        estop_button = RoundedButton(root, text="E-STOP", color="red", width=300, height=60, command=e_stop_cmd)
        estop_button.pack(side="bottom", padx=10, pady=10)
    

def create_round_button(canvas: tk.Canvas, width, height, cornerradius, color, offset = (0, 0)):
    shapes = []
    r = cornerradius
    shapes.append(canvas.create_polygon((offset[0], height-cornerradius+offset[1],
                                         offset[0], cornerradius+offset[1],
                                         cornerradius+offset[0], offset[1],
                                         width-cornerradius+offset[0], offset[1],
                                         width+offset[0], cornerradius+offset[1],
                                         width+offset[0], height-cornerradius+offset[1],
                                         width-cornerradius+offset[0], height+offset[1],
                                         cornerradius+offset[0], height+offset[1]),
                                         fill=color, outline=color))
    
    shapes.append(canvas.create_arc((offset[0],offset[1]+2*r,offset[0]+2*r,offset[1]), start=90, extent=90, fill=color, outline=color))
    shapes.append(canvas.create_arc((width+offset[0]-2*r,offset[1],width+offset[0],offset[1]+2*r), start=0, extent=90, fill=color, outline=color))
    shapes.append(canvas.create_arc((width+offset[0],height-2*r+offset[1],width+offset[0]-2*r,height+offset[1]), start=270, extent=90, fill=color, outline=color))
    shapes.append(canvas.create_arc((offset[0],height+offset[1]-2*r,offset[0]+2*r,height+offset[1]), start=180, extent=90, fill=color, outline=color))

    return shapes

class RoundedButton(tk.Canvas):
    def __init__(self, parent, text, image_path=None, color="grey", width=100, height=30, bg=dark2, command=None, cornerradius=6, enabled=True):
        tk.Canvas.__init__(self, parent, highlightthickness=0, bg=bg)
        self.command = command

        self.c_normal, self.c_hover, self.c_border_light, self.c_border_dark = button_colors[color]

        self.dark_bg = create_round_button(self, width, height, cornerradius, self.c_border_dark)
        self.light_bg = create_round_button(self, width-1, height-1, cornerradius, self.c_border_light)

        self.button = create_round_button(self, width-2, height-2, cornerradius, self.c_normal, offset=(1, 1))
        self.image_path = image_path

        self.configure(width=width+1, height=height+1)
        if enabled:
            self.bind("<ButtonPress-1>", self._on_press)
            self.bind("<ButtonRelease-1>", self._on_release)
            self.bind("<Enter>", self._on_enter)
            self.bind("<Leave>", self._on_leave)

            if self.image_path:
                self.image = tk.PhotoImage(file=self.image_path)
                self.image_id = self.create_image(width / 2, height / 2, image=self.image)
            else:
                self.label = self.create_text(width/2, height/2, text=text, font=('Roboto', 10, 'bold'), fill=text_white, anchor="center")
       
        else:
            for shape in self.button:
                self.itemconfig(shape, fill=self.c_border_dark, outline=self.c_border_dark)
            for shape in self.light_bg:
                self.itemconfig(shape, fill=self.c_border_dark, outline=self.c_border_dark)

            self.label = self.create_text(width/2, height/2, text=text, font=('Roboto', 10, 'bold'), fill=text_light_grey, anchor="center")

    def _on_enter(self, event):
        for shape in self.button:
            self.itemconfig(shape, fill=self.c_hover, outline=self.c_hover)
    
    def _on_leave(self, event):
        for shape in self.button:
            self.itemconfig(shape, fill=self.c_normal, outline=self.c_normal)

    def _on_press(self, event):
        for shape in self.light_bg:
            self.move(shape, 1, 1)
        if self.image_path:
            self.move(self.image, 1, 1)
        else:
            self.move(self.label, 1, 1)
        
    def _on_release(self, event):
        for shape in self.light_bg:
            self.move(shape, -1, -1)
        if self.image_path:
            self.move(self.image, -1, -1)
        else:
            self.move(self.label, -1, -1)

        if self.command is not None:
            self.command()
    
    def enable(self):
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)

        for shape in self.button:
            self.itemconfig(shape, fill=self.c_normal, outline=self.c_normal)
        for shape in self.light_bg:
            self.itemconfig(shape, fill=self.c_border_light, outline=self.c_border_light)

        self.itemconfig(self.label, fill=text_white)
