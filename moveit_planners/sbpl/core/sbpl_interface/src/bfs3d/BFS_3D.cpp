#include <sbpl_interface/bfs3d/BFS_3D.h>

namespace sbpl_interface
{
inline int BFS_3D::getNode(int x, int y, int z)
{
  if (x < 0 || y < 0 || z < 0 || x >= dim_x - 2 || y >= dim_y - 2 || z >= dim_z - 2)
  {
    // error "Invalid coordinates"
    return -1;
  }
  return (z + 1) * dim_xy + (y + 1) * dim_x + (x + 1);
}

BFS_3D::BFS_3D(int width, int height, int length)
{
  if (width <= 0 || height <= 0 || length <= 0)
  {
    // error "Invalid dimensions"
    return;
  }

  dim_x = width + 2;
  dim_y = height + 2;
  dim_z = length + 2;

  dim_xy = dim_x * dim_y;
  dim_xyz = dim_xy * dim_z;

  distance_grid = new int[dim_xyz];
  queue = new int[width * height * length];

  for (int node = 0; node < dim_xyz; node++)
  {
    int x = node % dim_x, y = node / dim_x % dim_y, z = node / dim_xy;
    if (x == 0 || x == dim_x - 1 || y == 0 || y == dim_y - 1 || z == 0 || z == dim_z - 1)
      distance_grid[node] = WALL;
    else
      distance_grid[node] = UNDISCOVERED;
  }

  running = false;
}

BFS_3D::~BFS_3D()
{
  if (search_thread_)
  {
    search_thread_->interrupt();
    search_thread_->join();
  }

  delete[] distance_grid;
  delete[] queue;
}

void BFS_3D::getDimensions(int* width, int* height, int* length)
{
  *width = dim_x - 2;
  *height = dim_y - 2;
  *length = dim_z - 2;
}

void BFS_3D::setWall(int x, int y, int z)
{
  if (running)
  {
    // error "Cannot modify grid while search is running"
    return;
  }

  int node = getNode(x, y, z);
  distance_grid[node] = WALL;
}

bool BFS_3D::isWall(int x, int y, int z)
{
  int node = getNode(x, y, z);
  return distance_grid[node] == WALL;
}

void BFS_3D::run(int x, int y, int z)
{
  if (running)
  {
    // error "Search already running"
    return;
  }

  for (int i = 0; i < dim_xyz; i++)
    if (distance_grid[i] != WALL)
      distance_grid[i] = UNDISCOVERED;

  origin = getNode(x, y, z);

  queue_head = 0;
  queue_tail = 1;
  queue[0] = origin;

  distance_grid[origin] = 0;

  search_thread_.reset(
      new boost::thread(&BFS_3D::search, this, dim_x, dim_xy, distance_grid, queue, queue_head, queue_tail));
  running = true;
}

int BFS_3D::getDistance(int x, int y, int z)
{
  int node = getNode(x, y, z);
  while (running && distance_grid[node] < 0)
    ;
  return distance_grid[node];
}
}
