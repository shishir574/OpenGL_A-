#include <iostream>
#include <glew.h>
#include <glfw3.h>
#include <time.h>
#include "a_star.h"
using namespace std;

//level size
#define width_ 1280
#define height_ 720

void exit_key(GLFWwindow* window, int key,int scancode, int isPressed, int mods)
{
    if (key == GLFW_KEY_ESCAPE && isPressed == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

float scale_x(float x)
{
    float x_;
    const float worldsize_X = (float)width_;
    const float range = 2.0f;

    x_ = (x / worldsize_X) * range - 1.0f;
    return x_;
}
float scale_y(float y)
{
    float y_;
    const float worldsize_Y = (float)height_;
    const float range = 2.0f;

    y_ = (y / worldsize_Y) * range - 1.0f;
    return y_;
}

int main(void)
{
    AStar::Generator generator;
    AStar::CoordinateList obstables;

    unsigned int num_obstacles = 0;

    //set origin and destination
    int origin_x = -1, origin_y = -1;
    int dest_x = -1, dest_y = -1;

    cout << "\tA-Star path finder and visualizer";
    cout << "\n\nPath method : Manhattan Distance.";
    cout << "\nOctogonal direction: True.";
    cout << "\nLevel size: 1280 * 720.";
    cout << "\nObstacles are generated at random.";

    cout << "\nSet number of obstacles (int): ";
    cin >> num_obstacles;

    cout << "\nEnter origin (x,y) : ";
    cin >> origin_x >> origin_y;

    cout << "\nEnter destination (x,y): ";
    cin >> dest_x >> dest_y;

    clock_t start = clock();
    //generate random obstacles
    for (int i = 0; i < num_obstacles; i++)
    {
        AStar::Vec2i coord = { rand() % width_, rand() % height_ };
        obstables.push_back(coord);
    }
    
    //generate level
    generator.setWorldSize({ width_, height_ },obstables);
    generator.setHeuristic(AStar::Heuristic::manhattan);
    generator.setDiagonalMovement(true);

    //find path
    auto path = generator.findPath({ origin_x, origin_y }, { dest_x, dest_y });

    if (path.empty())
    {
        cout << "No path found";
        return 0;
    }
    
    clock_t end = clock();

    cout << "\nTime taken to generate path: "<< (double)(end - start) / CLOCKS_PER_SEC<<"sec";

    GLFWwindow* window;

    if (!glfwInit())
        return -1;

    window = glfwCreateWindow(width_, height_, "ADA_A_STAR", NULL, NULL);
    
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, exit_key);
    
    //render level
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT);

        //draw level
        glLineWidth(4.0f);
        glPointSize(2.0f);

        glBegin(GL_POINTS); //obstacles
            for (size_t i = 0; i < obstables.size() ; i++)
            {
                glColor3f(1.0f, 1.0f, 1.0f);
                glVertex2f(scale_x(obstables[i].x), scale_y(obstables[i].y));
            }
        glEnd(); //end obstacles

        glBegin(GL_LINES); //path
            for (size_t i = 0; i < path.size() - 1; i++)
            {
                if (i <= path.size()-2 && i >= path.size() - 5)
                {
                    glColor3f(0.0f, 1.0f, 0.0f);
                    glVertex2f(scale_x(path[i].x), scale_y(path[i].y));
                    glVertex2f(scale_x(path[i + 1].x), scale_y(path[i + 1].y));
                }
                else if (i >= 0 && i <= 3)
                {
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glVertex2f(scale_x(path[i].x), scale_y(path[i].y));
                    glVertex2f(scale_x(path[i + 1].x), scale_y(path[i + 1].y));
                }
                else
                {
                    glColor3f(0.0f, 0.0f, 1.0f);
                    glVertex2f(scale_x(path[i].x), scale_y(path[i].y));
                    glVertex2f(scale_x(path[i + 1].x), scale_y(path[i + 1].y));
                }
            }
        glEnd(); //end path

        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}