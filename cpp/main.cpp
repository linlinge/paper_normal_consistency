#include <math.h>
#include <stdlib.h>
#include "PCLExtend.h"
#include "mouseManager.h"
pcl::PointCloud<PointType>::Ptr cloud;
static GLfloat theta[] = {0.0, 0.0, 0.0};
static GLint axis = 2;
float* dat;
void myReshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (w <= h)
        glOrtho(-2.0, 2.0, -2.0 * (GLfloat)h / (GLfloat)w,
                2.0 * (GLfloat)h / (GLfloat)w, -10.0, 10.0);
    else
        glMatrixMode(GL_MODELVIEW);
    glOrtho(-2.0 * (GLfloat)w / (GLfloat)h, 2.0 * (GLfloat)w / (GLfloat)h, -2.0,
            2.0, -10.0, 10.0);
}
GLfloat vertices[] = {-1.0, -1.0, 1.0, -1.0, 0.0, 1.0};
GLuint vertexBuffer;



void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //最小体素的边长
    float resolution = 10;
    pcl::octree::OctreePointCloudSearch<PointType> octree(resolution);
    octree.setInputCloud(cloud);

    // 从输入点云构建八叉树
    octree.addPointsFromInputCloud();

    //求出体素边界
    int depth = octree.getTreeDepth();
    for (auto it = octree.begin(depth);it != octree.end();++it){
        if (it.isLeafNode()){
            // Get minimum and maximum boundary of each voxel
            Eigen::Vector3f  voxel_min, voxel_max;
            octree.getVoxelBounds(it, voxel_min, voxel_max);
            
            vector<Vector3f> c=get_corners(voxel_min, voxel_max);
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_LINES); glVertex3f(c[0][0],c[0][1],c[0][2]); glVertex3f(c[1][0],c[1][1],c[1][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[1][0],c[1][1],c[1][2]); glVertex3f(c[2][0],c[2][1],c[2][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[2][0],c[2][1],c[2][2]); glVertex3f(c[3][0],c[3][1],c[3][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[3][0],c[3][1],c[3][2]); glVertex3f(c[0][0],c[0][1],c[0][2]); glEnd();

            glBegin(GL_LINES); glVertex3f(c[4][0],c[4][1],c[4][2]); glVertex3f(c[5][0],c[5][1],c[5][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[5][0],c[5][1],c[5][2]); glVertex3f(c[6][0],c[6][1],c[6][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[6][0],c[6][1],c[6][2]); glVertex3f(c[7][0],c[7][1],c[7][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[7][0],c[7][1],c[7][2]); glVertex3f(c[4][0],c[4][1],c[4][2]); glEnd();

            glBegin(GL_LINES); glVertex3f(c[0][0],c[0][1],c[0][2]); glVertex3f(c[4][0],c[4][1],c[4][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[1][0],c[1][1],c[1][2]); glVertex3f(c[5][0],c[5][1],c[5][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[2][0],c[2][1],c[2][2]); glVertex3f(c[6][0],c[6][1],c[6][2]); glEnd();
            glBegin(GL_LINES); glVertex3f(c[3][0],c[3][1],c[3][2]); glVertex3f(c[7][0],c[7][1],c[7][2]); glEnd();
          
        }
    }
    glColor3f(1.0f, 1.0f, 1.0f);
    glDrawArrays(GL_POINTS, 0, 3*cloud->points.size());
    glFlush();
    glutSwapBuffers();
}
void Init() {
    // load data
    cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    pcl::io::loadPLYFile("/home/llg/dataset/dataset_synthetic/11_armadillo_raw.ply",
                         *cloud);
    dat = new float[ 3 * cloud->points.size()];
    for (int i = 0; i < cloud->points.size(); i++) {
        dat[3 * i + 0] = cloud->points[i].x;
        dat[3 * i + 1] = cloud->points[i].y;
        dat[3 * i + 2] = cloud->points[i].z;
    }


    glewInit();
    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * cloud->points.size(), dat,
                 GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glEnable(GL_VERTEX_ARRAY);
}

int main(int argc, char** argv) {
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInit(&argc, argv);
    /* need both double buffering and z buffer */
    glutInitWindowSize(1000, 1000);
    glutCreateWindow("colorcube");
    glutReshapeFunc(myReshape);
    glutDisplayFunc(display);
    // glutIdleFunc(spinCube);
    glutMouseFunc(mouseCB);
    glutMotionFunc(mouseAndandoCB2);
    glEnable(GL_DEPTH_TEST); /* Enable hidden--surface--removal */
    Init();
    glutMainLoop();
    return 0;
}