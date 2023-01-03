#include <iostream>
#include "geometry.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream> 
#include <vector>


void genPolySphere(const float& radius, const int& subparts, const int& facesVertices, std::vector<Vec3f> &vertices)
{
    //subparts is a number of subparts a sphere is made of
    //face's Vertices is a number of vertices making up a polygon(face)

    int nVertices = 2 + facesVertices * (subparts - 1);
    std::unique_ptr<Vec3f[]> P {new Vec3f[nVertices]};

    // alpha is an angle between Y(up) axis and radius-vector at face's vertex.
    // betta is an angle between Z axis and projected radius-vector on XZ plane.

    float alpha, betta,da,db;
    alpha = 0;
    betta = 0;
    da = M_PI / subparts;
    db = 2 * M_PI / facesVertices;
    P[0] = Vec3f{ 0,radius,0 };
    int k = 1;
    for (int i = 0; i < subparts-1; ++i)
    {
        alpha += da;
        betta = M_PI;
        for (int j = 0; j < facesVertices; ++j)
        {
            float x = radius * sin(alpha) * sin(betta);
            float y = radius * cos(alpha);
            float z = radius * sin(alpha) * cos(betta);
            P[k] = Vec3f{ x,y,z };
            ++k;
            betta -= db;
        }
    }
    P[k] = Vec3f{ 0,-radius,0 };

    

    for (int i = 0; i < nVertices; ++i)
    {
        //std::cout << i << "(" << P[i].x << "," << P[i].y << "," << P[i].z << ")\n";
        vertices[i] = P[i];
    }
    
}

const float inchToMm = 25.4;
const float filmApertureWidth = 0.98 * inchToMm;
const float filmApertureHeight = 0.735 * inchToMm;
const float focalLength = 20;
const float near = 0.1;
const float far = 100;

const int width = 500;
const int height = 500;

void getNdcCoord(const Matrix44f& P, Vec3f& in, Vec3f& pNDC)

{

    pNDC.x = in.x * P[0][0] + in.y * P[1][0] + in.z * P[2][0] + /* in.z = 1 */ P[3][0];
    pNDC.y = in.x * P[0][1] + in.y * P[1][1] + in.z * P[2][1] + /* in.z = 1 */ P[3][1];
    pNDC.z = in.x * P[0][2] + in.y * P[1][2] + in.z * P[2][2] + /* in.z = 1 */ P[3][2];

    float w = in.x * P[0][3] + in.y * P[1][3] + in.z * P[2][3] + /* in.z = 1 */ P[3][3];

    if (w != 1)
    {
        pNDC.x /= w;
        pNDC.y /= w;
        pNDC.z /= w;
    }


}

void convertToRaster(const Vec3f& pNDC, Vec3f& pRaster, const int& iwidth, const int& iheight)
{

    pRaster.x = (pNDC.x + 1) / 2 * iwidth;
    pRaster.y = (1 - pNDC.y) / 2 * iheight;
    pRaster.z = pNDC.z;
}


int main()
{
    Matrix44f tmp = Matrix44f(0.707107, -0.331295, 0.624695, 0, 0, 0.883452, 0.468521, 0, -0.707107, -0.331295, 0.624695, 0, -1.63871, -5.747777, -40.400412, 1);
    std::vector<Vec3f> vertices;
    int subdiv = 80;
    int facesVertsNum = 80;
    vertices.resize(2+facesVertsNum*(subdiv-1));
    genPolySphere(1.5, subdiv, facesVertsNum,vertices);


    //Setting World to Camera Matrix
    Matrix44f worldToCamera;
    worldToCamera[3][1] = 0;
    worldToCamera[3][2] = 8;

    //Setting framebuffer
    unsigned char* framebuffer = new unsigned char[width * height];
    memset(framebuffer, 0x0, width * height);


    //building PerpProj Matrix we need n , f , t , b , r, l
    float right = (filmApertureWidth / 2) * near / focalLength;
    float top = (filmApertureHeight / 2) * near / focalLength;
    float left, bottom;

    const float filmAspectRatio = filmApertureWidth / filmApertureHeight;
    const float imageDeviceAspectRatio = width / float(height);

    float xoffset = 1;
    float yoffset = 1;

    if (filmAspectRatio > imageDeviceAspectRatio)
    {
        xoffset = imageDeviceAspectRatio / filmAspectRatio;
        yoffset = 1 / xoffset;
    }
    if (filmAspectRatio < imageDeviceAspectRatio)
    {
        xoffset = filmAspectRatio / imageDeviceAspectRatio;
        yoffset = 1 / xoffset;
    }

    right *= xoffset;
    top *= yoffset;
    left = -right;
    bottom = -top;

    Matrix44f P;
    P[0][0] = 2 * near / (right - left);
    P[1][1] = 2 * near / (top - bottom);
    P[2][0] = (right + left) / (right - left);
    P[2][1] = (top + bottom) / (top - bottom);
    P[2][2] = -(far + near) / (far - near);
    P[2][3] = -1;
    P[3][2] = -2 * far * near / (far - near);
    P[3][3] = 1;

    for (int i = 0; i < vertices.size(); ++i)
    {
        Vec3f pCamera;
        Vec3f pWorld = vertices[i];
        worldToCamera.multVecMatrix(pWorld, pCamera);

        Vec3f pNDC;
        getNdcCoord(P, pCamera, pNDC);

        if (pNDC.x > 1 || pNDC.x < -1 || pNDC.y>1 || pNDC.y < -1)
            continue;

        Vec3f pRaster;
        convertToRaster(pNDC, pRaster, width, height);
        uint32_t x = std::min(width - 1, int(pRaster.x));
        uint32_t y = std::min(height - 1, int(pRaster.y));
        framebuffer[y * width + x] = 255;


    }

    for (int i = 0; i < vertices.size(); ++i)
    {
        //std::cout << i << "(" << P[i].x << "," << P[i].y << "," << P[i].z << ")\n";
        std::cout<<vertices[i].x<<","<<vertices[i].y<<","<<vertices[i].z<<'\n';
    }
    std::ofstream ofs;

    ofs.open("proced_Sphere19.ppm", std::iostream::binary);
    ofs << "P5\n" << width << " " << height << "\n255\n";
    ofs.write((char*)framebuffer, width * height);
    ofs.close();
    delete[] framebuffer;

    return 0;
}

