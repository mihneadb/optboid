import collections
from math import floor


class BoidSwarm(object):

    """
    The grid to hold the boids
    """

    def __init__(self, width, cell_w):
        """
        Create data structure to hold the things. At the base is a table of cell nodes
        each containing an arbitrary number of units. Need whole number of cells, so cell width
        is the total width (for now in pixel units) divded by the divisions.
        The structure is always square. It can provide boundaries though good level design
        should mean you don't ever hit them.
        """

        self.boids = []  # list of all the boids

        divs = int(floor(width/cell_w))
        self.divisions = divs
        self.cell_width = cell_w

        self.cell_table = {}
        self.num_cells = divs*divs
        for i in range(divs):
            for j in range(divs):
                # use deque for fast appends of several deque
                self.cell_table[(i, j)] = collections.deque()

    def cell_num(self, x, y):
        """Forces units into border cells if they hit an edge"""
        i = int(floor(x / self.cell_width))
        j = int(floor(y / self.cell_width))
        # deal with boundary conditions
        if i < 0:
            i = 0
        if j < 0:
            j = 0
        if i >= self.divisions:
            i = self.divisions-1  # zero indexing
        if j >= self.divisions:
            j = self.divisions-1  # zero indexing
        return (i, j)

    def find_cell_containing(self, x, y):
        """returns the cell containing a position x,y"""
        return self.cell_table[self.cell_num(x, y)]

    def find_near(self, x, y, influence_range):
        """return objects within radius influence_range of point x,y"""
        if influence_range == 0:
            _nearObjects = self.find_cell_containing(x, y)
        elif influence_range <= self.cell_width:
            _nearObjects = self.find_neighbour_cells(x, y)
        else:
            ext = ceil(influence_range/self.cell_width)
            _nearObjects = self.find_extended(x, y, ext)

        return _nearObjects

    def find_neighbour_cells(self, x, y):
        return self.cell_table[self.cell_num(x, y)]

    def find_extended(self, x, y, d):
        """
        use to find set of cells surrounding the cell containing position x,y
        """
        I, J = self.cell_num(x, y)
        d = int(d)
        group = collections.deque()
        for i in range(I-d, I+d):
            for j in range(J-d, J+d):
                if (i, j) in self.cell_table:
                    group.extend(self.cell_table[(i, j)])  # merge deque
        return group

    def rebuild(self):
        for cell in self.cell_table.values():
            cell.clear()
        for b in self.boids:
            c = self.find_cell_containing(b.position.x, b.position.y)
            c.append(b)
