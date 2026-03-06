import matplotlib
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


class Draw:
    def __init__(self, axis_lim, gap, chess_grid):
        self.axis_lim = axis_lim
        self.gap = gap
        self.grid = chess_grid
        self.projection_ratio = chess_grid[0]/axis_lim
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        plt.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.98, wspace=0)
        self.draw_concircle()
        self.ax.set_aspect('equal')
        self.ax.spines['left'].set_position('zero')
        self.ax.spines['bottom'].set_position('zero')
        self.ax.set_xlim([-axis_lim, axis_lim])
        self.ax.set_ylim([-axis_lim, axis_lim])
        # Set the x and y ticks to be the same
        self.ax.set_xticks(np.arange(-axis_lim, axis_lim, gap))
        self.ax.set_yticks(np.arange(-axis_lim, axis_lim, gap))
        self.ax.text(0.5, 1.01, 'N', transform=self.ax.transAxes, va='bottom', ha='center')
        # plt.grid()

    def draw_arrow(self, xy, uv, color, alpha=1):
        x, y = xy
        u, v = uv
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        if xlim[0] <= x <= xlim[1] and ylim[0] <= y <= ylim[1]:
            self.ax.arrow(x, y, u, v, head_width=0.1, head_length=0.1, fc=color, ec=color, alpha=alpha)
            self.ax.scatter(x, y, color=color,s = 35, alpha=alpha)

    def draw_circle(self, center, radius, color, alpha):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        if xlim[0] <= center[0] <= xlim[1] and ylim[0] <= center[1] <= ylim[1]:
            circle = Circle(center, radius, color=color, alpha=alpha, fill=False)
            self.ax.add_patch(circle)

    def draw_concircle(self, alpha=0.3):
        for radius in range(0, self.axis_lim * 2, self.gap):
            circle = Circle((0, 0), radius, fill=False, alpha=alpha)
            self.ax.add_patch(circle)

    # given a list of coordinates, draw lines that connect them in order
    def draw_path_lines(self, coords, color, alpha=0.5):
        for i in range(len(coords)-1):
            self.ax.plot([coords[i][0], coords[i+1][0]], [coords[i][1], coords[i+1][1]], color=color, alpha=alpha)
            self.ax.scatter(coords[i][0], coords[i][1], color=color,s = 35, alpha=alpha)
        self.ax.scatter(coords[-1][0], coords[-1][1], color=color,s = 35, alpha=alpha)

    def draw_path_points(self, path, color="green"):
        x_path = tuple([x/self.projection_ratio-self.axis_lim for x, y in path])
        y_path = tuple([y/self.projection_ratio-self.axis_lim for x, y in path])
        self.ax.scatter(x_path,y_path, color=color,alpha = 0.5, s=5)
    
    def clean_up(self):
        for patch in self.ax.patches:
            patch.remove()
        for coll in self.ax.collections:
            if isinstance(coll, matplotlib.collections.PathCollection):
                coll.remove()
        for line in self.ax.lines:
            line.remove()
        for text in self.ax.texts:
            text.remove()
    
    def clean_up_ex_text(self):
        for patch in self.ax.patches:
            patch.remove()
        for coll in self.ax.collections:
            if isinstance(coll, matplotlib.collections.PathCollection):
                coll.remove()
        for line in self.ax.lines:
            line.remove()

    def draw_text(self, coords, text, color="black", alpha=0.5):
        x, y = coords
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        if xlim[0] <= x <= xlim[1] and ylim[0] <= y <= ylim[1]:
            self.ax.text(x, y, text, color=color, alpha=alpha)
