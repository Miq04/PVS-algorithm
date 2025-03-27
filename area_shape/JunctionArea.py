import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from utils import write_pickle

def add(a,b):
    return (a[0]+b[0], a[1]+b[1])

class Junction:
    def __init__(self, x, y):
        self.center = (x, y)
        self.lane_dict = {}

    def generate_edges(self, num_entry, num_exit, edge_length=500.0, num_lanes=3):
        entry_edges = [f"-E{i}" for i in range(num_entry)]
        exit_edges = [f"E{i}" for i in range(num_exit)]

        for edge_id in entry_edges:
            self.add_edge(edge_id, edge_length, num_lanes, usage = "entry")

        for edge_id in exit_edges:
            self.add_edge(edge_id, edge_length, num_lanes, usage = "exit")

    def add_edge(self, edge_id, edge_length, num_lanes, usage, lane_width=3.2):
        for i in range(num_lanes):
            lane_id = f"{edge_id}_{i}"
            lane_shape = self.generate_lane_shape(edge_length, edge_id, i, lane_width, usage)
            self.lane_dict[lane_id] = lane_shape

    def generate_lane_shape(self, edge_length, edge_id, lane_index, lane_width, usage = ""):
        if usage == "exit":
            sign = 1
        elif usage == "entry":
            sign = -1
        shape = [0] * 4
        lane_width_num = 2 - lane_index
        if "0" in edge_id:
            shape[0] = add((-13.6, sign*lane_width_num*lane_width), self.center)
            shape[1] = (shape[0][0], shape[0][1] + sign*lane_width)
            shape[2] = (-edge_length, shape[1][1])
            shape[3] = (shape[2][0], shape[2][1] - sign*lane_width)
            shape = [(round(coord[0], 1), round(coord[1], 1)) for coord in shape]
            assert shape[0][1] == shape[3][1], "ArithmeticError"
        
        elif "3" in edge_id:
            shape[2] = add((13.6, -sign*lane_width_num*lane_width), self.center)
            shape[3] = (shape[2][0], shape[2][1] - sign*lane_width)
            shape[0] = (edge_length, shape[3][1])
            shape[1] = (shape[0][0], shape[0][1] + sign*lane_width)
            shape = [(round(coord[0], 1), round(coord[1], 1)) for coord in shape]
            assert shape[2][1] == shape[1][1], "ArithmeticError"

        elif "1" in edge_id:
            shape[2] = add((sign*lane_width_num*lane_width, 13.6), self.center)
            shape[3] = (shape[2][0] + sign*lane_width, shape[2][1])
            shape[0] = (shape[3][0], edge_length)
            shape[1] = (shape[0][0]- sign*lane_width, edge_length)
            shape = [(round(coord[0], 1), round(coord[1], 1)) for coord in shape]
            assert shape[2][0] == shape[1][0], "ArithmeticError"

        elif "2" in edge_id:
            shape[0] = add((-sign*lane_width_num*lane_width, -13.6),self.center)
            shape[1] = (shape[0][0]- sign*lane_width, shape[0][1])
            shape[2] = (shape[1][0], -edge_length)
            shape[3] = (shape[2][0] + sign*lane_width, -edge_length)
            shape = [(round(coord[0], 1), round(coord[1], 1)) for coord in shape]
            assert shape[0][0] == shape[3][0], "ArithmeticError"
        
        return shape
    
    def generate_grids(self, grid_len):
        grids_dict = {}

        def find_start_end(points, x_or_y):
            start = min(points, key=lambda x: abs(x[x_or_y]))
            end = max(points, key=lambda x: abs(x[x_or_y]))
            return round(start[x_or_y], 1), round(end[x_or_y], 1)

        for lane_id, points in self.lane_dict.items():
            if 'E0' in lane_id or 'E3' in lane_id:
                # horizental
                x_or_y = 0
                start, end = find_start_end(points, x_or_y)
            elif 'E1' in lane_id or 'E2' in lane_id:
                # vertical
                x_or_y = 1 
                start, end = find_start_end(points, x_or_y)
            else:
                raise ValueError(f"Invalid lane orientation:{lane_id}")

            segments_count = abs(int((end - start) / grid_len))

            for i in range(segments_count):
                if 'E0' in lane_id:
                    segment_positions = [round(start - i * grid_len, 1) for i in range(segments_count)]
                    x_start, x_end = segment_positions[i], segment_positions[i] - grid_len
                    y_min, y_max = find_start_end(points, 1)
                elif 'E1' in lane_id:
                    segment_positions = [round(start + i * grid_len, 1) for i in range(segments_count)]
                    y_start, y_end = segment_positions[i], segment_positions[i] + grid_len
                    x_min, x_max = find_start_end(points, 0)
                elif 'E2' in lane_id:
                    segment_positions = [round(start - i * grid_len, 1) for i in range(segments_count)]
                    y_start, y_end = segment_positions[i], segment_positions[i] - grid_len
                    x_min, x_max = find_start_end(points, 0)
                elif 'E3' in lane_id:
                    segment_positions = [round(start + i * grid_len, 1) for i in range(segments_count)]
                    x_start, x_end = segment_positions[i], segment_positions[i] + grid_len
                    y_min, y_max = find_start_end(points, 1)
                else:
                    raise ValueError(f"Invalid lane orientation:{lane_id}")

                new_lane_id = f"{lane_id}_{i}"

                if not x_or_y:
                    new_points = [(round(x_start, 1), round(y_min, 1)),
                                    (round(x_end, 1), round(y_min, 1)),
                                    (round(x_end, 1), round(y_max, 1)),
                                    (round(x_start, 1), round(y_max, 1))]
                else:
                    new_points = [(round(x_min, 1), round(y_start, 1)),
                                    (round(x_min, 1), round(y_end, 1)),
                                    (round(x_max, 1), round(y_end, 1)),
                                    (round(x_max, 1), round(y_start, 1))]

                grids_dict[new_lane_id] = new_points

        return grids_dict

if __name__ == "__main__":
    # ------------------create junction shape dict------------------  
    junction = Junction(0, 0)
    junction.generate_edges(num_entry=4, num_exit=4)
    lane_dict = junction.lane_dict
    grids_dict = junction.generate_grids(grid_len=4.6)
    write_pickle(lane_dict, "data/pkl/lane_dict.pkl")
    write_pickle(grids_dict, "data/pkl/grids_dict.pkl")

    # ------------------junction area visualize------------------
    fig, ax = plt.subplots()
    for label, line in lane_dict.items():
        polygon = Polygon(line, closed=True, edgecolor='black', facecolor='none')
        ax.add_patch(polygon)
        ax.plot(*zip(*line), marker='o', label=label)

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.legend()
    ax.axis('equal')
    plt.savefig('area_shape/JunctionArea.png', dpi=300)
    plt.show()
