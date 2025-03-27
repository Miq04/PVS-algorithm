import pickle

def save_dict(obj, path):
    assert path.endswith('.pkl'), "path must ends with .pkl"
    with open(path, 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_dict(path):
    with open(path, 'rb') as f:
        return pickle.load(f)

def write_pickle(obj, path):
    assert path.endswith('.pkl'), "path must ends with .pkl"
    with open(path, 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def read_pickle(path):
    with open(path, 'rb') as f:
        return pickle.load(f)

def search_all_vehicle(t, data):
    all_vehicle_pos_at_t = []
    all_vehicle_at_t = data[data['t'] == t]
    if len(all_vehicle_at_t):
        for row in all_vehicle_at_t.itertuples():
            all_vehicle_pos_at_t.append((row.veh_x, row.veh_y))
    return all_vehicle_pos_at_t
