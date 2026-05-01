#!/usr/bin/env python3

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


ROBOT_NS = '/robot_15'


class GameStateDemoGui(Node):
    def __init__(self):
        super().__init__('game_state_demo_gui')

        self.event_pub = self.create_publisher(
            String, f'{ROBOT_NS}/game_event', 10
        )
        self.light_pub = self.create_publisher(
            String, f'{ROBOT_NS}/game_light', 10
        )
        self.sound_pub = self.create_publisher(
            String, f'{ROBOT_NS}/game_sound', 10
        )

        self.root = tk.Tk()
        self.root.title('Pac-ManBot Game State Effects Demo')
        self.root.geometry('960x700')
        self.root.minsize(860, 620)
        self.root.configure(bg='#050514')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.motion_enabled = tk.BooleanVar(value=False)
        self.status_text = tk.StringVar(value='Ready. Motion effects are off.')

        self.pacman_items = []
        self.ghost_items = []
        self.pellet_items = []
        self.path_points = []
        self.animation_step = 0

        self.configure_styles()
        self.build_ui()
        self.root.after(80, self.animate_scene)
        self.root.after(20, self.spin_ros)

        self.get_logger().info('Game state demo GUI ready')

    def configure_styles(self):
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure(
            'Pac.TButton',
            background='#11114a',
            foreground='#ffe766',
            bordercolor='#2336ff',
            focusthickness=2,
            focuscolor='#ffe766',
            padding=(12, 8),
            font=('TkDefaultFont', 10, 'bold')
        )
        style.map(
            'Pac.TButton',
            background=[('active', '#1f2cff'), ('pressed', '#ffe766')],
            foreground=[('pressed', '#050514')]
        )
        style.configure(
            'Danger.TButton',
            background='#4a1016',
            foreground='#ff8c8c',
            bordercolor='#ff3333',
            padding=(12, 8),
            font=('TkDefaultFont', 10, 'bold')
        )
        style.map(
            'Danger.TButton',
            background=[('active', '#7a151f'), ('pressed', '#ff3333')],
            foreground=[('pressed', '#050514')]
        )
        style.configure(
            'Pac.TCheckbutton',
            background='#050514',
            foreground='#ffe766',
            font=('TkDefaultFont', 11, 'bold')
        )
        style.map('Pac.TCheckbutton', background=[('active', '#050514')])

    def build_ui(self):
        header = tk.Frame(self.root, bg='#050514')
        header.pack(fill='x', padx=18, pady=(14, 8))

        title = tk.Label(
            header,
            text='PAC-MANBOT EFFECTS',
            bg='#050514',
            fg='#ffe766',
            font=('TkDefaultFont', 24, 'bold')
        )
        title.pack(side='left')

        motion_toggle = ttk.Checkbutton(
            header,
            text='Enable motion',
            variable=self.motion_enabled,
            style='Pac.TCheckbutton',
            command=self.update_motion_status
        )
        motion_toggle.pack(side='right', padx=(12, 0))

        self.canvas = tk.Canvas(
            self.root,
            height=210,
            bg='#050514',
            highlightthickness=3,
            highlightbackground='#243cff'
        )
        self.canvas.pack(fill='x', padx=18, pady=(0, 14))
        self.canvas.bind('<Configure>', self.redraw_scene)

        scroll_shell = tk.Frame(self.root, bg='#050514')
        scroll_shell.pack(fill='both', expand=True, padx=18, pady=(0, 12))

        controls_canvas = tk.Canvas(
            scroll_shell,
            bg='#050514',
            highlightthickness=0,
            borderwidth=0
        )
        controls_scrollbar = ttk.Scrollbar(
            scroll_shell,
            orient='vertical',
            command=controls_canvas.yview
        )
        controls_canvas.configure(yscrollcommand=controls_scrollbar.set)

        controls_canvas.pack(side='left', fill='both', expand=True)
        controls_scrollbar.pack(side='right', fill='y')

        content = tk.Frame(controls_canvas, bg='#050514')
        content_window = controls_canvas.create_window(
            (0, 0),
            window=content,
            anchor='nw'
        )

        def update_scroll_region(_event=None):
            controls_canvas.configure(scrollregion=controls_canvas.bbox('all'))

        def fit_content_width(event):
            controls_canvas.itemconfigure(content_window, width=event.width)

        def scroll_controls(event):
            if event.num == 4:
                controls_canvas.yview_scroll(-1, 'units')
            elif event.num == 5:
                controls_canvas.yview_scroll(1, 'units')
            else:
                controls_canvas.yview_scroll(
                    int(-1 * (event.delta / 120)),
                    'units'
                )

        content.bind('<Configure>', update_scroll_region)
        controls_canvas.bind('<Configure>', fit_content_width)
        controls_canvas.bind_all('<MouseWheel>', scroll_controls)
        controls_canvas.bind_all('<Button-4>', scroll_controls)
        controls_canvas.bind_all('<Button-5>', scroll_controls)

        left = tk.Frame(content, bg='#050514')
        left.pack(side='left', fill='both', expand=True, padx=(0, 12))

        right = tk.Frame(content, bg='#050514')
        right.pack(side='left', fill='both', expand=True, padx=(12, 0))

        self.add_button_grid(
            left,
            'Mapped Game Events',
            [
                ('Start', lambda: self.publish_event('start')),
                ('Start Theatrical', lambda: self.publish_motion_aware(
                    'start theatrical',
                    [('sound', 'pacman_theme'), ('light', 'start')]
                )),
                ('Pellet', lambda: self.publish_event('pellet')),
                ('Power Pellet', lambda: self.publish_event('power pellet')),
                ('Blinky Caught', lambda: self.publish_event('blinky caught')),
                ('Pinky Caught', lambda: self.publish_event('pinky caught')),
                ('Inky Caught', lambda: self.publish_event('inky caught')),
                ('Clyde Caught', lambda: self.publish_event('clyde caught')),
                ('Blinky Killed', lambda: self.publish_motion_aware(
                    'blinky killed',
                    [('light', 'blinky killed'), ('sound', 'death')]
                )),
                ('Pinky Killed', lambda: self.publish_motion_aware(
                    'pinky killed',
                    [('light', 'pinky killed'), ('sound', 'death')]
                )),
                ('Inky Killed', lambda: self.publish_motion_aware(
                    'inky killed',
                    [('light', 'inky killed'), ('sound', 'death')]
                )),
                ('Clyde Killed', lambda: self.publish_motion_aware(
                    'clyde killed',
                    [('light', 'clyde killed'), ('sound', 'death')]
                )),
                ('Game Over', lambda: self.publish_motion_aware(
                    'game over',
                    [('light', 'game over'), ('sound', 'death')]
                )),
                ('Win', lambda: self.publish_event('win')),
                ('Reset', lambda: self.publish_event('reset')),
            ],
            columns=3
        )

        self.add_button_grid(
            right,
            'Direct Light Tests',
            [
                ('Pac-Man', lambda: self.publish_light('pacman')),
                ('Pellet', lambda: self.publish_light('pellet')),
                ('Power Pellet', lambda: self.publish_light('power pellet')),
                ('Win', lambda: self.publish_light('win')),
                ('Blinky', lambda: self.publish_light('blinky caught')),
                ('Pinky', lambda: self.publish_light('pinky caught')),
                ('Inky', lambda: self.publish_light('inky caught')),
                ('Clyde', lambda: self.publish_light('clyde caught')),
                ('Blinky Killed', lambda: self.publish_light('blinky killed')),
                ('Pinky Killed', lambda: self.publish_light('pinky killed')),
                ('Inky Killed', lambda: self.publish_light('inky killed')),
                ('Clyde Killed', lambda: self.publish_light('clyde killed')),
                ('Yellow', lambda: self.publish_light('yellow')),
                ('Red', lambda: self.publish_light('red')),
                ('Blue', lambda: self.publish_light('blue')),
                ('Off', lambda: self.publish_light('off')),
            ],
            columns=2
        )

        self.add_button_grid(
            right,
            'Direct Audio Tests',
            [
                ('Start', lambda: self.publish_sound('start')),
                ('Theme', lambda: self.publish_sound('pacman_theme')),
                ('Pellet', lambda: self.publish_sound('pellet')),
                ('Power Pellet', lambda: self.publish_sound('power pellet')),
                ('Ghost', lambda: self.publish_sound('ghost')),
                ('Death', lambda: self.publish_sound('death')),
                ('Win', lambda: self.publish_sound('win')),
            ],
            columns=2
        )

        status = tk.Label(
            self.root,
            textvariable=self.status_text,
            anchor='w',
            bg='#050514',
            fg='#ffffff',
            font=('TkDefaultFont', 10)
        )
        status.pack(fill='x', padx=18, pady=(0, 12))

    def add_button_grid(self, parent, title, buttons, columns):
        section = tk.Frame(parent, bg='#050514')
        section.pack(fill='x', pady=(0, 16))

        label = tk.Label(
            section,
            text=title,
            bg='#050514',
            fg='#7cc7ff',
            font=('TkDefaultFont', 13, 'bold')
        )
        label.grid(
            row=0, column=0, columnspan=columns, sticky='w', pady=(0, 8)
        )

        for index, (text, command) in enumerate(buttons):
            row = index // columns + 1
            column = index % columns
            is_danger = 'Killed' in text or text == 'Game Over'
            style = 'Danger.TButton' if is_danger else 'Pac.TButton'
            button = ttk.Button(
                section, text=text, command=command, style=style
            )
            button.grid(row=row, column=column, sticky='ew', padx=4, pady=4)

        for column in range(columns):
            section.columnconfigure(column, weight=1, uniform=title)

    def redraw_scene(self, _event=None):
        self.canvas.delete('all')
        width = max(self.canvas.winfo_width(), 600)
        height = max(self.canvas.winfo_height(), 180)
        margin = 28

        self.canvas.create_rectangle(
            margin,
            margin,
            width - margin,
            height - margin,
            outline='#243cff',
            width=4
        )
        self.canvas.create_rectangle(
            margin + 18,
            margin + 18,
            width - margin - 18,
            height - margin - 18,
            outline='#243cff',
            width=2
        )

        self.path_points = self.make_border_path(
            width, height, margin + 36, step=18
        )
        self.pellet_items = []
        for index, (x_pos, y_pos) in enumerate(self.path_points):
            if index % 2 == 0:
                item = self.canvas.create_oval(
                    x_pos - 3,
                    y_pos - 3,
                    x_pos + 3,
                    y_pos + 3,
                    fill='#ffe766',
                    outline=''
                )
                self.pellet_items.append((index, item))

        self.canvas.create_text(
            width / 2,
            height / 2,
            text='SELECT AN EFFECT',
            fill='#ffe766',
            font=('TkDefaultFont', 22, 'bold')
        )
        self.canvas.create_text(
            width / 2,
            height / 2 + 32,
            text='audio  lights  event mapping',
            fill='#ffffff',
            font=('TkDefaultFont', 12, 'bold')
        )

        self.draw_characters()

    def make_border_path(self, width, height, margin, step):
        points = []
        left = margin
        right = width - margin
        top = margin
        bottom = height - margin

        for x_pos in range(left, right + 1, step):
            points.append((x_pos, top))
        for y_pos in range(top + step, bottom + 1, step):
            points.append((right, y_pos))
        for x_pos in range(right - step, left - 1, -step):
            points.append((x_pos, bottom))
        for y_pos in range(bottom - step, top, -step):
            points.append((left, y_pos))

        return points

    def animate_scene(self):
        if self.path_points:
            self.animation_step = (
                self.animation_step + 1
            ) % len(self.path_points)
            self.draw_characters()
        self.root.after(90, self.animate_scene)

    def draw_characters(self):
        for item in self.pacman_items + self.ghost_items:
            self.canvas.delete(item)
        self.pacman_items = []
        self.ghost_items = []

        if not self.path_points:
            return

        pac_index = self.animation_step % len(self.path_points)
        pac_x, pac_y = self.path_points[pac_index]
        next_index = (pac_index + 1) % len(self.path_points)
        next_x, next_y = self.path_points[next_index]
        angle = math.degrees(math.atan2(next_y - pac_y, next_x - pac_x))
        mouth = 35 if pac_index % 2 == 0 else 8
        self.pacman_items.append(self.canvas.create_arc(
            pac_x - 15,
            pac_y - 15,
            pac_x + 15,
            pac_y + 15,
            start=angle + mouth,
            extent=360 - 2 * mouth,
            fill='#ffd21f',
            outline='#ffd21f'
        ))

        ghost_specs = [
            (10, '#ff3333'),
            (15, '#ff70c8'),
            (20, '#36c9ff'),
            (25, '#ff9b2f'),
        ]
        for offset, color in ghost_specs:
            ghost_index = (pac_index - offset) % len(self.path_points)
            ghost_x, ghost_y = self.path_points[ghost_index]
            self.ghost_items.extend(self.draw_ghost(ghost_x, ghost_y, color))

        for index, item in self.pellet_items:
            state = 'hidden' if abs(index - pac_index) < 3 else 'normal'
            self.canvas.itemconfigure(item, state=state)

    def draw_ghost(self, x_pos, y_pos, color):
        items = [
            self.canvas.create_arc(
                x_pos - 13,
                y_pos - 13,
                x_pos + 13,
                y_pos + 13,
                start=0,
                extent=180,
                fill=color,
                outline=color
            ),
            self.canvas.create_rectangle(
                x_pos - 13,
                y_pos,
                x_pos + 13,
                y_pos + 13,
                fill=color,
                outline=color
            ),
        ]
        for offset in (-8, 0, 8):
            items.append(self.canvas.create_polygon(
                x_pos + offset - 5,
                y_pos + 13,
                x_pos + offset,
                y_pos + 7,
                x_pos + offset + 5,
                y_pos + 13,
                fill='#050514',
                outline='#050514'
            ))
        for eye_x in (-5, 5):
            items.append(self.canvas.create_oval(
                x_pos + eye_x - 3,
                y_pos - 4,
                x_pos + eye_x + 3,
                y_pos + 2,
                fill='white',
                outline=''
            ))
        return items

    def update_motion_status(self):
        if self.motion_enabled.get():
            self.status_text.set(
                'Motion effects enabled. Event mappings publish cmd_vel.'
            )
        else:
            self.status_text.set(
                'Motion effects are off. Motion events use direct audio/light.'
            )

    def publish_motion_aware(self, event_name, fallback_commands):
        if self.motion_enabled.get():
            self.publish_event(event_name)
            return

        for command_type, command in fallback_commands:
            if command_type == 'light':
                self.publish_light(command, update_status=False)
            elif command_type == 'sound':
                self.publish_sound(command, update_status=False)

        self.status_text.set(
            f'{event_name}: lights/audio only because motion is disabled.'
        )
        self.get_logger().info(
            'Published direct fallback for motion-sensitive event: '
            f'{event_name}'
        )

    def publish_event(self, event_name):
        self.publish(self.event_pub, event_name)
        self.status_text.set(f'Published event: {event_name}')

    def publish_light(self, command, update_status=True):
        self.publish(self.light_pub, command)
        if update_status:
            self.status_text.set(f'Published light command: {command}')

    def publish_sound(self, command, update_status=True):
        self.publish(self.sound_pub, command)
        if update_status:
            self.status_text.set(f'Published sound command: {command}')

    def publish(self, publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)
        self.get_logger().info(f'Published demo command: {text}')

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(20, self.spin_ros)

    def run(self):
        self.root.mainloop()

    def close(self):
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    node = GameStateDemoGui()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
