#include <cmath>
#include <vector>
#include <stdlib.h>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace operations_research {




struct DataModel {
  std::vector<std::vector<int64>> distance_matrix; /*{
      {0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468,
       776, 662},
      {548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674,
       1016, 868, 1210},
      {776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130,
       788, 1552, 754},
      {696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822,
       1164, 560, 1358},
      {582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708,
       1050, 674, 1244},
      {274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514,
       1050, 708},
      {502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514,
       1278, 480},
      {194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662,
       742, 856},
      {308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320,
       1084, 514},
      {194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274,
       810, 468},
      {536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730,
       388, 1152, 354},
      {502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308,
       650, 274, 844},
      {388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536,
       388, 730},
      {354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342,
       422, 536},
      {468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342,
       0, 764, 194},
      {776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388,
       422, 764, 0, 798},
      {662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536,
       194, 798, 0},
  };*/
    const std::vector<std::vector<RoutingIndexManager::NodeIndex>>
      pickups_deliveries{
          {RoutingIndexManager::NodeIndex{1},
           RoutingIndexManager::NodeIndex{6}},
          {RoutingIndexManager::NodeIndex{2},
           RoutingIndexManager::NodeIndex{10}},
          {RoutingIndexManager::NodeIndex{4},
           RoutingIndexManager::NodeIndex{3}},
          {RoutingIndexManager::NodeIndex{5},
           RoutingIndexManager::NodeIndex{9}},
          {RoutingIndexManager::NodeIndex{7},
           RoutingIndexManager::NodeIndex{8}},
          {RoutingIndexManager::NodeIndex{15},
           RoutingIndexManager::NodeIndex{11}},
          {RoutingIndexManager::NodeIndex{13},
           RoutingIndexManager::NodeIndex{12}},
          {RoutingIndexManager::NodeIndex{16},
           RoutingIndexManager::NodeIndex{14}},
      };
  int num_vehicles = 4;
  RoutingIndexManager::NodeIndex depot{0};
  std::vector<int64> demands{
    0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8,
  };
  std::vector<int64> vehicle_capacities{30, 30, 30, 30};
};



//! @brief Print the solution.
//! @param[in] data Data of the problem.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
void VRPPrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) {
  int64 max_route_distance{0};
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int64 index = routing.Start(vehicle_id);
    LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
    int64 route_distance{0};
    std::stringstream route;
    while (routing.IsEnd(index) == false) {
      route << manager.IndexToNode(index).value() << " -> ";
      int64 previous_index = index;
      index = solution.Value(routing.NextVar(index));
      route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     int64{vehicle_id});
    }
    LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    LOG(INFO) << "Distance of the route: " << route_distance << "m";
    max_route_distance = std::max(route_distance, max_route_distance);
  }
  LOG(INFO) << "Maximum of the route distances: " << max_route_distance << "m";
  LOG(INFO) << "";
  LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}

void VrpGlobalSpan() {
  std::cout << "Subbash running: wait for a few seconds" << std::endl;
  // Time window example from or tools documentation is not yet run here
  // Also duration taken to load and unload is missing here

  // Instantiate the data problem.
  DataModel data;
  
  const int size = 17;
  srand (42);
  
  for (int i = 0; i < size; i++) {
    std::vector<int64> row;
    for (int j = 0; j < size; j++) {
      if (i == j) {
        row.push_back(0);
      } else if (i > j) {
        row.push_back(data.distance_matrix[j][i]);
      } else {
        row.push_back(rand() % 1000);
      }
    }
    data.distance_matrix.push_back(row);
  }
  
  
  
  for (std::vector<int64> row : data.distance_matrix) {
    for (int64 val : row) {
      std::cout << val << " ";
    }
    std::cout << std::endl;
  }

  // Create Routing Index Manager
  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);

  // Create Routing Model.
  RoutingModel routing(manager);

  // Create and register a transit callback.
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](int64 from_index, int64 to_index) -> int64 {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager.IndexToNode(from_index).value();
        auto to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });
  
  const int demand_callback_index = routing.RegisterUnaryTransitCallback(
    [&data, &manager](int64 from_index) -> int64 {
      // Convert from routing variable Index to demand NodeIndex.
      int from_node = manager.IndexToNode(from_index).value();
      return data.demands[from_node];
    });

  // Define cost of each arc.
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Add Distance constraint.
  routing.AddDimension(transit_callback_index, 0,
                       3000,  // maximum distance per vehicle?
                       true,  // start cumul to zero
                       "Distance");
  RoutingDimension* distance_dimension = routing.GetMutableDimension("Distance");
  distance_dimension->SetGlobalSpanCostCoefficient(100);

  // More general compared to AddDimension
  routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,    // transit callback index
    int64{0},                 // null capacity slack
    data.vehicle_capacities,  // vehicle maximum capacities
    true,                     // start cumul to zero
    "Capacity");

  
  Solver* const solver = routing.solver();

  // Define Transportation Requests.
  /*
  for (const auto& request : data.pickups_deliveries) {
    int64 pickup_index = manager.NodeToIndex(request[0]);
    int64 delivery_index = manager.NodeToIndex(request[1]);
    routing.AddPickupAndDelivery(pickup_index, delivery_index);
    solver->AddConstraint(solver->MakeEquality(
        routing.VehicleVar(pickup_index), routing.VehicleVar(delivery_index)));
    solver->AddConstraint(
        solver->MakeLessOrEqual(distance_dimension->CumulVar(pickup_index),
                                distance_dimension->CumulVar(delivery_index)));
  }
  */

  // Setting first solution heuristic.
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  // searchParameters.set_first_solution_strategy(
  //    FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
  searchParameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  //searchParameters.set_local_search_metaheuristic(
  //    LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  //searchParameters.mutable_time_limit()->set_seconds(4);
  //searchParameters.set_log_search(true);



  // Solve the problem.
  // std::cout << "next solution is" << routing.solver()->NextSolution() << std::endl;
  
  const Assignment* solution = routing.SolveWithParameters(searchParameters);

  // Print solution on console.
  if (routing.status() == 1) {
    VRPPrintSolution(data, manager, routing, *solution);
  }
  std::cout << "solver status is : " << routing.status() << std::endl;
  std::cout << "number of solution found are: " << routing.solver()->solutions() << std::endl;
  
  // Check if solution is not nullptr first else it will have segmentation fault
  //std::cout << solution << std::endl;

  
  /*
  std::vector<std::vector<int>> answer = GetRoutes(manager, routing, *solution);
  for (std::vector<int> row : answer) {
    for (int i : row) {
      std::cout << i << " - ";
    }
    std::cout << std::endl;
  }
  */
}



}  // namespace operations_research

int main(int argc, char** argv) {
  //operations_research::Tsp2();
  operations_research::VrpGlobalSpan();
  return EXIT_SUCCESS;
}
