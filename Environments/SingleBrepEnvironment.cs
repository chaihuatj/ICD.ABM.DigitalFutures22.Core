using ABxM.Core.Environments;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace ICD.ABM.DigitalFutures22.Core.Environments
{
    /// <summary>
    /// The class provides the environment for agent systems on (trimmed) surfaces.
    /// </summary>
    public class SingleBrepEnvironment : SurfaceEnvironment
    {
        public Brep BrepObject;

        /// <summary>
        /// The field that defines a 2D-array of curvature nodes, which are a custom structure defined in this class.
        /// </summary>
        public CurvatureNode[,] CurvatureNodes;

        /// <summary>
        /// Constructs a new, empty instance of the single brep environment.
        /// </summary>
        public SingleBrepEnvironment()
        {
        }

        /// <summary>
        /// Constructs a new instance of the single brep environment based on a given brep and a spacing value for computing the curvature field.
        /// </summary>
        /// <param name="brep">The brep that defines the environment.</param>
        /// <param name="spacing">The double defining the spacing of the environment's curvature field.</param>
        public SingleBrepEnvironment(Brep brep, double spacing)
        {
            BrepObject = brep;
            ComputeCurvatureField(spacing);
        }

        /// <summary>
        /// Method for setting the breps of the environment. Q: Shouldn't this rather be done using a set-property?
        /// </summary>
        /// <param name="brep">The brep to be set as the environment's base brep.</param>
        public void SetBrep(Brep brep)
        {
            BrepObject = brep;
        }

        /// <summary>
        /// A method to compute a 2d-array of curvature values from the brep based on a given spacing parameter.
        /// </summary>
        /// <param name="spacing">The double defining the curvature field's spacing.</param>
        public void ComputeCurvatureField(double spacing)
        {
            //_resolution = R
            //_cols = _width / _resolution
            //_rows = _height / _resolution
            Surface srf = BrepObject.Surfaces[0];
            srf.SetDomain(0, new Interval(0.0, 1.0));
            srf.SetDomain(1, new Interval(0.0, 1.0));

            int cols = (int)(srf.IsoCurve(0, srf.Domain(0).Mid).GetLength() / spacing);
            int rows = (int)(srf.IsoCurve(0, srf.Domain(1).Mid).GetLength() / spacing);

            CurvatureNodes = new CurvatureNode[cols + 1, rows + 1];
            double s = double.NaN;
            double t = double.NaN;
            ComponentIndex ci = ComponentIndex.Unset;
            Vector3d normal = Vector3d.Unset;
            Point3d p = Point3d.Unset;

            for (int i = 0; i <= cols; i++)
            {
                for (int j = 0; j <= rows; j++)
                {
                    Point3d uv = new Point3d(i / (double)cols, j / (double)rows, 0);
                    if (BrepObject.ClosestPoint(
                            srf.PointAt(uv.X, uv.Y),
                            out p, out ci, out s, out t,
                            Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,
                            out normal))
                    {
                        //CurvatureNodes[i, j].UV = uv;
                        Vector3d[] derivatives;
                        srf.Evaluate(uv.X, uv.Y, 2, out p, out derivatives);
                        //CurvatureNodes[i, j].Position = p;
                        CurvatureNodes[i, j].FirstDerivative = derivatives[0];
                        CurvatureNodes[i, j].SecondDerivative = derivatives[1];
                        CurvatureNodes[i, j].SurfaceCurvature = srf.CurvatureAt(uv.X, uv.Y);
                    }
                    else
                    {
                        CurvatureNodes[i, j].FirstDerivative = Vector3d.Unset;
                        CurvatureNodes[i, j].SecondDerivative = Vector3d.Unset;
                        CurvatureNodes[i, j].SurfaceCurvature = null;
                    }
                }
            }
        }

        /// <summary>
        /// Method for resetting the environment.
        /// </summary>
        public override void Reset()
        {
        }

        /// <summary>
        /// Method for collecting the DisplayGeometries.
        /// </summary>
        /// <returns>The list of objects to be displayed.</returns>
        public override List<object> GetDisplayGeometry()
        {
            List<object> surfaceCurvatures = new List<object>();

            for (int i = 0; i < CurvatureNodes.GetLength(0); i++)
                for (int j = 0; j < CurvatureNodes.GetLength(1); j++)
                    surfaceCurvatures.Add(CurvatureNodes[i, j].SurfaceCurvature);

            return surfaceCurvatures;
        }

        /// <inheritdoc/>
        public override Point3d GetClosestPoint(Point3d position)
        {
            return BrepObject.ClosestPoint(position);
        }

        /// <inheritdoc/>
        public override Vector3d GetNormal(Point3d position)
        {
            return GetTangentPlane(position).Normal;
        }

        /// <inheritdoc/>
        public override Plane GetTangentPlane(Point3d position)
        {
            Surface surface = BrepObject.Surfaces[0];
            double u, v;
            surface.ClosestPoint(position, out u, out v);
            Plane plane;
            surface.FrameAt(u, v, out plane);
            return plane;
        }

        /// <inheritdoc/>
        public override Point3d IntersectWithLine(Line line)
        {
            Point3d[] intPts = Rhino.Geometry.Intersect.Intersection.RayShoot(new Ray3d(line.From, line.Direction), new List<GeometryBase>() { BrepObject }, 1);

            // output
            if (intPts == null || intPts.Length == 0) return Point3d.Unset;
            return intPts[0];
        }

        /// <summary>
        /// Method for getting the 2D boundary curves of the brep environment.
        /// </summary>
        /// <returns>Returns the list of curves that the define the 2D boundaries of the environment.</returns>
        public List<Curve> BoundaryCurves2D()
        {
            List<Curve> boundaryCurves = new List<Curve>();
            boundaryCurves.AddRange(BrepObject.Curves2D);
            return boundaryCurves;
        }

        /// <inheritdoc/>
        public override List<Curve> GetBoundaryCurves3D()
        {
            List<Curve> boundaryCurves = new List<Curve>();
            boundaryCurves.AddRange(BrepObject.Curves3D);
            return boundaryCurves;
        }

        /// <summary>
        /// Method for getting the UV coordinates of the closest point on the brep for a given reference point, e.g. the position of the agent.
        /// </summary>
        /// <param name="position">The test point.</param>
        /// <returns>Returns a Point3D composed of the UVW coordinates of the closest point to the test point on the environment.</returns>
        public Point3d UVCoordinates(Point3d position)
        {
            Surface surface = BrepObject.Surfaces[0];
            double u, v;
            surface.ClosestPoint(position, out u, out v);
            return new Point3d(u, v, 0.0);
        }
    }

    /// <summary>
    /// A custom structure that holds the first and second derivate vectors and Rhino's SurfaceCurvature object.
    /// </summary>
    public struct CurvatureNode
    {
        /// <summary>
        /// A 3D vector representing the direction of the slope (1st derivative).
        /// </summary>
        public Vector3d FirstDerivative;
        /// <summary>
        /// A 3D vector representing the direction of the change of slope.
        /// </summary>
        public Vector3d SecondDerivative;
        /// <summary>
        /// Rhino's SurfaceCurvature object.
        /// </summary>
        public SurfaceCurvature SurfaceCurvature;
    }
}
