#include "surf.h"
#include "extra.h"
using namespace std;

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }
}

static void triangulate(vector< Tup3u >& list, int u, int v) {
    for( int i = 0; i < u; i++ ) {
        for( int j = 0; j < v; j++ ) {
            Tup3u face1;
            face1[0] = i * v + j;
            face1[1] = i * v + j + 1;
            face1[2] = (i + 1) * v + j;
            list.push_back(face1);
            Tup3u face2;
            face2[0] = i * v + j + 1;
            face2[1] = (i + 1) * v + j + 1;
            face2[2] = (i + 1) * v + j;
            list.push_back(face2); 
        }
    }
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    float step_size = 6.28 / steps;
    for( float i = 0; i <= 6.28; i+=step_size ) {
        Matrix3f rotation = Matrix3f::rotateY(i);
        for( int j = 0; j < profile.size(); j++ ) {
            surface.VV.push_back(rotation * profile[j].V);
            surface.VN.push_back(-(rotation * profile[j].N)); 
        }
    }

    triangulate(surface.VF, steps, profile.size());
 
    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }


    for( int i = 0; i < sweep.size(); i++ ) {
        Vector4f sweep_N = Vector4f(sweep[i].N, 0);
        Vector4f sweep_B = Vector4f(sweep[i].B, 0);
        Vector4f sweep_T = Vector4f(sweep[i].T, 0);
        Vector4f sweep_V = Vector4f(sweep[i].V, 1);
        Matrix4f vertex_transform = Matrix4f(sweep_N, sweep_B, sweep_T, sweep_V);
        Matrix3f normal_transform = vertex_transform.getSubmatrix3x3(0, 0).transposed().inverse();
        for( int j = 0; j < profile.size(); j++ ) {
            Vector4f point = Vector4f(profile[j].V, 1);
            Vector3f normal = profile[j].N;
            surface.VV.push_back((vertex_transform * point).xyz());
            surface.VN.push_back(-(normal_transform * normal));
        }
}

    triangulate(surface.VF, sweep.size(), profile.size());
 
    return surface;
}

void drawSurface(const Surface &surface, bool shaded)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {        
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        
        glColor4f(0.4f,0.4f,0.4f,1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

void drawNormals(const Surface &surface, float len)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1);
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i=0; i<surface.VV.size(); i++)
    {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (unsigned i=0; i<surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i=0; i<surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
