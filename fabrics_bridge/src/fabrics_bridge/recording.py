import pickle
from copy import deepcopy
import numpy as np

class Recording():
    def __init__(self, tf_matrix):
        self._data = []
        self._tf_matrix = tf_matrix
        self._name = "not_named"
        self._indices = {
            'position_1': [0, 3],
            'position_2': [3, 6],
            'position_3': [6, 9],
            'header': 9,
            'weight_1': 10,
            'weight_2': 11,
            'weight_3': 12,
            'tolerance_1': 13,
            'tolerance_2': 14,
            'tolerance_3': 15,
            'vacuum': 16,
            'part_present': 17,
        }
        self._record_threshold = 0.0005

    def name(self, name: str) -> None:
        self._name = name

    def get_name(self) -> str:
        return self._name + ".pickle"

    def number_waypoints(self) -> int:
        return len(self._data)


    def append_waypoint(self, waypoint: list) -> None:
        if len(self._data) < 1:
            distance = 1
        else:
            distance = np.linalg.norm(
                np.array(waypoint[0:3]) - np.array(self._data[-1][0:3])
            )
            vacuum_changed = self._data[-1][self._indices['vacuum']] != waypoint[self._indices['vacuum']]
        if (distance >= self._record_threshold) or vacuum_changed:
            self._data.append(waypoint)
        else:
            print("Waypoint discarded because it is too close to the old one.")

    def remove_waypoint(self, index: int) -> None:
        self._data.pop(index)

    def transform_waypoints(self, tf_matrix) -> None:
        self._transformed_data = deepcopy(self._data)
        for i in range(len(self._data)):
            for j in range(1,4):
                index_0 = self._indices[f'position_{j}'][0]
                index_1 = self._indices[f'position_{j}'][1]
                ow = self._data[i][index_0:index_1]

                if j == 1:
                    extending_entry = 1
                elif j == 2 or j == 3:
                    extending_entry = 0
                aw = np.array([ow[0], ow[1], ow[2], extending_entry])
                naw = np.dot(tf_matrix, aw)
                nw = naw[0:3]
                self._transformed_data[i][index_0:index_1] = nw


    def waypoints(self) -> list:
        if not hasattr(self, '_transformed_data'):
            self._transformed_data = deepcopy(self._data)
        return self._transformed_data

    def tf_matrix(self) -> np.ndarray:
        return self._tf_matrix

    def set_tf_matrix(self, tf_matrix) -> None:
        self._tf_matrix = tf_matrix

    def values(self, name: str) -> np.ndarray:
        start_index = self._indices[name][0]
        stop_index = self._indices[name][1]
        return np.array([d[start_index: stop_index] for d in self._data])

    def smoothen_trajectory(self, window_size: int=20):

        names = ['position_1', 'position_2', 'position_3']
        for name in names:
            start_index = self._indices[name][0]
            stop_index = self._indices[name][1]
            values = self.values(name)
            smooth_values = np.zeros_like(values)
            values= np.pad(values, [[window_size//2, window_size//2-1], [0, 0]], mode='edge')

            for i in range(3):
                smooth_values[:, i] = np.convolve(values[:, i], np.ones(window_size)/window_size, mode='valid')

            for i in range(len(self._data)):
                self._data[i][start_index: stop_index] = smooth_values[i, 0:3].tolist()

    def save(self, smoothen: bool = False):
        if smoothen:
            self.smoothen_trajectory()
        with open(f"{self._name}.pickle", "wb") as f:
            pickle.dump(self, f)
        print(f"Saved {len(self._data)} poses to {self._name}.pickle")

    def __str__(self) -> str:
        return str(np.array(self._data)[:, 1])




