import time
import gurobipy as gp
from gurobipy import GRB
from utils import read_pickle, write_pickle

grids_dict = read_pickle('data/pkl/grids_dict.pkl')
grids_keys = list(grids_dict.keys())

def previous_vehicle_selection(vehicle_ids, previous_selected_vehicle_ids):
    previous_vehicle_ids_index = []
    vehicle_ids = list(vehicle_ids)
    for vehicle_id in vehicle_ids:
        if vehicle_id in previous_selected_vehicle_ids:
            previous_vehicle_ids_index.append(vehicle_ids.index(vehicle_id))
    return previous_vehicle_ids_index

def optimize_global_vehicle_selection(detection_matrix, in_junction_matrix, previous_vehicle_indices, max_communication_num, grids_keys=grids_keys):
    num_grids = len(detection_matrix)
    num_vehicles, num_periods = detection_matrix[0].shape
    start_time = time.time()
    model = gp.Model("Global_Vehicle_Selection_Weights")

    # 决策变量: x[i] 表示车辆 i 是否被选中
    x = model.addVars(num_vehicles, vtype=GRB.BINARY, name="x")

    # 辅助变量: r[m][j] 表示栅格 m 在时间 j 至少有一个车辆探测
    r = model.addVars(num_grids, num_periods, vtype=GRB.BINARY, name="r")

    if len(previous_vehicle_indices) > 0:
        # 辅助变量: z[i] 表示车辆 i 是否被连续选中
        z = model.addVars(num_vehicles, vtype=GRB.BINARY, name="z")

    # 目标函数 1：最小未覆盖
    not_detection_penalty = gp.quicksum((1-r[m, j]) * (20 if grids_keys[m].endswith(('_0', '_1', '_2', '_3', '_4')) else 1) 
                                for m in range(num_grids) for j in range(num_periods))
    not_detection_penalty = not_detection_penalty / num_grids / num_periods
    z1 = model.addVar(name="z1")
    model.addConstr(z1 == not_detection_penalty + 1)
    log_z1 = model.addVar(name="log_z1")
    model.addGenConstrLog(z1, log_z1, "log_not_detection_penalty")
    model.setObjectiveN(100*log_z1, index=0, priority=3)

    # 目标函数 2：最小通信成本
    normalized_communication_cost = x.sum() / max_communication_num
    z2 = model.addVar(name="z2")
    model.addConstr(z2 == normalized_communication_cost + 1)
    log_z2 = model.addVar(name="log_z2")
    model.addGenConstrLog(z2, log_z2, "log_communication_cost")
    model.setObjectiveN(100*log_z2, index=1, priority=2)

    # 目标函数 3：最小化重复探测的惩罚
    overlap_penalty = gp.quicksum(gp.quicksum(x[i] * detection_matrix[m][i, j] * (0.2 if grids_keys[m].endswith(('_0', '_1', '_2', '_3', '_4')) else 1) 
                              for i in range(num_vehicles)) for j in range(num_periods) for m in range(num_grids))
    overlap_penalty = overlap_penalty / (num_vehicles * num_periods * num_grids)
    z3 = model.addVar(name="z3")
    model.addConstr(z3 == overlap_penalty + 1)
    log_z3 = model.addVar(name="log_z3")
    model.addGenConstrLog(z3, log_z3, "log_overlap_penalty")
    model.setObjectiveN(100*log_z3, index=2, priority=1)
   
    # 目标函数 4：最大化通信稳定性
    stability_objective_terms = 0
    if len(previous_vehicle_indices) > 0:
        for i in range(num_vehicles):
            if i in previous_vehicle_indices:
                model.addConstr(z[i] == x[i], name=f"Stability_{i}")
                stability_objective_terms += (z[i] / max_communication_num)
        z4 = model.addVar(name="z4")
        model.addConstr(z4 == stability_objective_terms + 1)
        log_z4 = model.addVar(name="log_z4")
        model.addGenConstrLog(z4, log_z4, "log_stability_objective_terms")
        model.setObjectiveN(-100*log_z4, index=3, priority=0)

    # 约束: 每个时间点通信的车辆数量不超过max_communication_num
    for j in range(num_periods):
        model.addConstr(gp.quicksum(x[i] * in_junction_matrix[i, j] for i in range(num_vehicles)) <= max_communication_num, name=f"Bandwidth_{j}")
    
    M = num_vehicles
    # 约束: 尝试确保每个时间点至少有一辆车在探测
    for m in range(num_grids):
        for j in range(num_periods):
            # 如果探测车辆总数为0，则r[m][j]应该为0
            model.addConstr(gp.quicksum(x[i] * detection_matrix[m][i, j] for i in range(num_vehicles)) >= 1 * r[m, j], name=f"MinCoverageRequire_{m}_{j}")
            # 如果探测车辆总数大于0，则r[m][j]应该为1
            model.addConstr(gp.quicksum(x[i] * detection_matrix[m][i, j] for i in range(num_vehicles)) <= M * r[m, j], name=f"MinCoverageActivate_{m}_{j}") 

    # 求解模型
    model.optimize()
    used_time = time.time() - start_time

    if model.status == GRB.OPTIMAL:
        print("Optimal set of vehicles:", [i for i in range(num_vehicles) if x[i].X > 0.5])
    elif model.status == GRB.INFEASIBLE:
        print("Model is infeasible; consider relaxing some constraints.")

    return [x[i].X for i in range(num_vehicles)], used_time

def optimize_all_window_vehicle_selection(vehicle_ids_list, detection_matrices, in_junction_matrices, window_indices, max_communication_num):
    all_window_best_sol = []
    previous_selected_vehicle_ids = []
    solution_time = []
    for i in window_indices:
        vehicle_ids = vehicle_ids_list[i]
        detection_matrix = detection_matrices[i]
        in_junction_matrix = in_junction_matrices[i]
        previous_vehicle_indices = previous_vehicle_selection(vehicle_ids, previous_selected_vehicle_ids)
        window_sol, window_time = optimize_global_vehicle_selection(detection_matrix, in_junction_matrix, previous_vehicle_indices, max_communication_num)
        selected_vehicle_ids = [vehicle_ids[i] for i,x in enumerate(window_sol) if x > 0.5]
        all_window_best_sol.append(selected_vehicle_ids)
        solution_time.append(window_time)
        previous_selected_vehicle_ids = selected_vehicle_ids
    return all_window_best_sol, solution_time

if __name__ == "__main__":
    detection_radius1 = 20
    vehicle_ids_list = read_pickle("data/pkl/all_window_vehicle_ids_truth.pkl")
    detection_matrices = read_pickle(f"data/pkl/{detection_radius1}/detection_matrices_pre_{detection_radius1}.pkl")
    in_junction_matrices = read_pickle(f"data/pkl/{detection_radius1}/in_junction_matrices_pre_{detection_radius1}.pkl")

    detection_matrices = [detection_matrix[:1260, : , :] for detection_matrix in detection_matrices]

    window_indices = [0, 19]
    test_communication_num = list(range(1, 10)) + list(range(10, 50, 5)) + list(range(55, 166, 10))  # 29 samples
    
    # scenario 1
    windows_best_sols1, windows_sol_times1 = [], []
    for max_communication_num in test_communication_num:
        all_window_best_sol, solution_time = optimize_all_window_vehicle_selection(
            vehicle_ids_list, detection_matrices, in_junction_matrices, window_indices[:1], max_communication_num)
        windows_best_sols1.append(all_window_best_sol)
        windows_sol_times1.append(solution_time)
    sol_save_path = f"data/pkl/analysis/windows_best_sol_1c1.pkl"
    times_save_path = f"data/pkl/analysis/windows_sol_times_1c1.pkl"
    write_pickle(windows_best_sols1, sol_save_path)
    write_pickle(windows_sol_times1, times_save_path)

    # scenario 2
    windows_best_sols2, windows_sol_times2 = [], []
    for max_communication_num in test_communication_num:
        all_window_best_sol, solution_time = optimize_all_window_vehicle_selection(
            vehicle_ids_list, detection_matrices, in_junction_matrices, window_indices[1:], max_communication_num)
        windows_best_sols2.append(all_window_best_sol)
        windows_sol_times2.append(solution_time)
    sol_save_path = f"data/pkl/analysis/windows_best_sol_1c2.pkl"
    times_save_path = f"data/pkl/analysis/windows_sol_times_1c2.pkl"
    write_pickle(windows_best_sols2, sol_save_path)
    write_pickle(windows_sol_times2, times_save_path)
