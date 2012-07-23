#ifndef _SBPL_INTERFACE_BFS_3D_H_
#define _SBPL_INTERFACE_BFS_3D_H_

namespace sbpl_interface{
#define WALL         0x7FFFFFFF
#define UNDISCOVERED 0xFFFFFFFF
class BFS_3D {
    private:
        int dim_x, dim_y, dim_z;
        int dim_xy, dim_xyz;

        int origin;
        int volatile* distance_grid;

        int* queue;
        int queue_head, queue_tail;

        volatile bool running;

        void search(int, int, int volatile*, int*, int&, int&);
        inline int getNode(int, int, int);

    public:
        BFS_3D(int, int, int);
        ~BFS_3D();

        void getDimensions(int*, int*, int*);

        void setWall(int, int, int);
        bool isWall(int, int, int);

        void run(int, int, int);

        int getDistance(int, int, int);
};
}

#endif
