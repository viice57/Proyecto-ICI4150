#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define GRID_SIZE 10
#define CELL_SIZE 0.5
#define MAX_PATH_LEN 100
#define MAX_OPEN_NODES 1000
#define SPEED 4.0

typedef struct {
  int x, y;
} Point;

typedef struct {
  int x, y;
  int g, h, f;
  int parent_index;
} Node;


int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int plan_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int max_path_len) {
  Node open_list[MAX_OPEN_NODES];
  int open_count = 0;

  Node closed_list[GRID_SIZE * GRID_SIZE];
  int closed_count = 0;

  Node start_node = {start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), 0, -1};
  start_node.f = start_node.g + start_node.h;
  open_list[open_count++] = start_node;

  while (open_count > 0) {
    int best_index = 0;
    for (int i = 1; i < open_count; i++) {
      if (open_list[i].f < open_list[best_index].f)
        best_index = i;
    }

    Node current = open_list[best_index];
    for (int i = best_index; i < open_count - 1; i++)
      open_list[i] = open_list[i + 1];
    open_count--;
    closed_list[closed_count++] = current;

    if (current.x == goal.x && current.y == goal.y) {
      int length = 0;
      Node n = current;
      while (n.parent_index != -1 && length < max_path_len) {
        path[length++] = (Point){n.x, n.y};
        n = closed_list[n.parent_index];
      }
      path[length++] = (Point){start.x, start.y};
      for (int i = 0; i < length / 2; i++) {
        Point temp = path[i];
        path[i] = path[length - i - 1];
        path[length - i - 1] = temp;
      }
      return length;
    }

    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};
    for (int d = 0; d < 4; d++) {
      int nx = current.x + dx[d];
      int ny = current.y + dy[d];
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
        continue;
      if (grid[nx][ny] == 1)
        continue;

      int in_closed = 0;
      for (int i = 0; i < closed_count; i++) {
        if (closed_list[i].x == nx && closed_list[i].y == ny) {
          in_closed = 1;
          break;
        }
      }
      if (in_closed) continue;

      int g = current.g + 1;
      int h = heuristic(nx, ny, goal.x, goal.y);
      int f = g + h;

      int in_open = -1;
      for (int i = 0; i < open_count; i++) {
        if (open_list[i].x == nx && open_list[i].y == ny) {
          in_open = i;
          break;
        }
      }

      if (in_open != -1) {
        if (f < open_list[in_open].f) {
          open_list[in_open].g = g;
          open_list[in_open].h = h;
          open_list[in_open].f = f;
          open_list[in_open].parent_index = closed_count - 1;
        }
      } else if (open_count < MAX_OPEN_NODES) {
        Node neighbor = {nx, ny, g, h, f, closed_count - 1};
        open_list[open_count++] = neighbor;
      }
    }
  }
  return 0;
}

void recalculate_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int *path_length) {
    *path_length = plan_path(grid, start, goal, path, 100); // Usamos la función plan_path para recalcular la ruta
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
*/
 
int main() {
  wb_robot_init();
  double left_speed = 1.0;
  double right_speed = 1.0;
  int i;

  //motores
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  //sensores 
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  //lidar
  
  WbDeviceTag lidar=wb_robot_get_device("lidar");
  wb_lidar_enable(lidar,TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  
  //GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  //ruta grilla
  int grid[GRID_SIZE][GRID_SIZE] = {0};
  Point path[100];
  int path_length = 0;
  Point path[100];
  Point start = {GRID_SIZE / 2, GRID_SIZE / 2};
  Point goal = {GRID_SIZE - 2, GRID_SIZE - 2};
  
  // Generar la ruta inicial
  recalculate_path(grid, start, goal, path, &path_length);
  
  while (wb_robot_step(TIME_STEP) != -1) {
      // Obtener valores de sensores
      double ds_values[2];
      bool obstacle_detected = false;
      for (int i = 0; i < 2; i++) {
          ds_values[i] = wb_distance_sensor_get_value(ds[i]);
          if (ds_values[i] < 950.0) { // Umbral de detección de obstáculos
              obstacle_detected = true;
              break;
          }
      }
  
      // Obtener la posición actual del robot
      const double *pose = wb_gps_get_values(gps);
      float robot_x = pose[0];
      float robot_y = pose[2];
  
      // Lógica para manejar obstáculos
      if (obstacle_detected) {
          printf("Obstáculo detectado, recalculando ruta...\n");
          
          // Actualizar la cuadrícula con los datos más recientes del entorno (LIDAR y sensores de distancia)
          update_grid_with_sensor_data(grid);
  
          // Recalcular la ruta
          recalculate_path(grid, start, goal, path, &path_length);
      }
  
      // Seguir el camino calculado
      if (path_length > 1) {
          Point next_point = path[0]; // Obtener el siguiente punto en el camino
          float target_x = next_point.x * CELL_SIZE;
          float target_y = next_point.y * CELL_SIZE;
          
          // Aquí implementa el movimiento hacia el siguiente punto (esto es solo una representación simplificada)
          move_to_target(target_x, target_y);  // Implementa la lógica para mover al siguiente punto
      } else {
          // Si ya hemos alcanzado la meta
          left_speed = 0.0;
          right_speed = 0.0;
      }
  
      wb_motor_set_velocity(wheels[0], left_speed);
      wb_motor_set_velocity(wheels[1], right_speed);
      wb_motor_set_velocity(wheels[2], left_speed);
      wb_motor_set_velocity(wheels[3], right_speed);
  
      fflush(stdout); 
  };


  
  wb_robot_cleanup();

  return 0;
}
