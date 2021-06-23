#ifdef __APPLE__
#include <OpenGL/gl.h>
/* Just in case we need these later
// References:
// http://alumni.cs.ucsb.edu/~wombatty/tutorials/opengl_mac_osx.html
// # include <OpenGL/gl.h>
// # include <OpenGL/glu.h>
*/
#else
#include <GL/gl.h>
#endif

#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif
using namespace std;

namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }

}



Curve evalBezier( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
        cerr << "called with " << P.size() << " points." << endl;
        exit( 0 );
    }

    // TODO:
    // You should implement this function so that it returns a Curve
    // (e.g., a vector< CurvePoint >).  The variable "steps" tells you
    // the number of points to generate on each piece of the spline.
    // At least, that's how the sample solution is implemented and how
    // the SWP files are written.  But you are free to interpret this
    // variable however you want, so long as you can control the
    // "resolution" of the discretized spline curve with it.

    // Make sure that this function computes all the appropriate
    // Vector3fs for each CurvePoint: V,T,N,B.
    // [NBT] should be unit and orthogonal.

    // Also note that you may assume that all Bezier curves that you
    // receive have G1 continuity.  Otherwise, the TNB will not be
    // be defined at points where this does not hold.

    cerr << "\t>>> evalBezier has been called with the following input:" << endl;

    cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;
    
    //////////////////////////////////////////////////////////////////////
    // ==================== BEGIN IMPLEMENTATION ====================== //
    //////////////////////////////////////////////////////////////////////
    Curve curve = Curve();
    const vector<const Vector3f >::iterator it = P.begin();

    // Iterate through control points
    for( int i = 0; i < P.size() - 1; i += 3 )
    {
        // Set geometry matrix
        Matrix4f G_b = Matrix4f( Vector4f(it[i], 0), 
                                 Vector4f(it[i + 1], 0),
                                 Vector4f(it[i + 2], 0), 
                                 Vector4f(it[i + 3], 0), 
                                 true );

        // Set basis matrix
        const Matrix4f M_b = Matrix4f(
            1, -3,  3, -1, 
            0,  3, -6,  3, 
            0,  0,  3, -3,
            0,  0,  0, 1
        );

        Matrix4f G_b_M_b = G_b * M_b;    // geometry matrix * basis matrix

        Vector3f prev_binormal = Vector3f(0, 0, 1);
        // Compute CurvePoint for each step
        for( int j = 0; j <= steps; j++ ) {
            float t = (float)j / (float)steps;
            Vector4f T_b = Vector4f(1, t, t*t, t*t*t);
            Vector4f dT_b = Vector4f(0, 1, 2*t, 3*t*t);
            CurvePoint p;
            p.V = (G_b_M_b * T_b).xyz();
            p.T = (G_b_M_b * dT_b).xyz().normalized();
            
            // TODO: normal and binormal
            p.N = Vector3f::cross(prev_binormal, p.T);
            p.B = Vector3f::cross(p.T, p.N);
            prev_binormal = p.B;

            curve.push_back(p);
        }
    }

    return curve;
    //////////////////////////////////////////////////////////////////////
    // ===================== END IMPLEMENTATION ======================= //
    //////////////////////////////////////////////////////////////////////
}

Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 )
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit( 0 );
    }

    // TODO:
    // It is suggested that you implement this function by changing
    // basis from B-spline to Bezier.  That way, you can just call
    // your evalBezier function.

    cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

    cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;

    //////////////////////////////////////////////////////////////////////
    // ==================== BEGIN IMPLEMENTATION ====================== //
    //////////////////////////////////////////////////////////////////////
    Curve curve = Curve();
    const vector< const Vector3f >::iterator it = P.begin();

    const Matrix4f M_bezier = Matrix4f( 1, -3,  3, -1, 
                                        0,  3, -6,  3, 
                                        0,  0,  3, -3,
                                        0,  0,  0, 1 );

    const Matrix4f M_spline = Matrix4f( 1./6., -.5,  .5, -1./6.,
                                        4./6.,  0., -1.,     .5,
                                        1./6.,  .5,  .5,    -.5,
                                           0.,  0.,  0.,   1./6. );

    const Matrix4f M_bezier_inv = M_bezier.inverse();
    const Matrix4f transform = M_spline * M_bezier_inv;

    // Iterate through control points
    for( int i = 0; i < P.size() - 3; i++ )
    {
        vector< Vector3f > controlPoints; 

        // Set geometry matrix
        Matrix4f G_b = Matrix4f( Vector4f(it[i], 0), 
                                 Vector4f(it[i + 1], 0),
                                 Vector4f(it[i + 2], 0), 
                                 Vector4f(it[i + 3], 0), 
                                 true );

        const Matrix4f pts = G_b * transform;

        for( int j = 0; j < 4; j++ ) {
            Vector3f point = pts.getCol(j).xyz();
            controlPoints.push_back(point);
        }


        vector< Vector3f > const& const_controlPoints = controlPoints;

        Curve temp = evalBezier(const_controlPoints, steps);

        if (i < P.size() - 4) {
            temp.pop_back();
        }

        for(int j = 0; j < temp.size(); j++) {
            curve.push_back(temp.at(j));
        }
    }

    return curve;
    //////////////////////////////////////////////////////////////////////
    // ===================== END IMPLEMENTATION ======================= //
    //////////////////////////////////////////////////////////////////////
}

Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).
    
    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }
    
    // Pop state
    glPopAttrib();
}

