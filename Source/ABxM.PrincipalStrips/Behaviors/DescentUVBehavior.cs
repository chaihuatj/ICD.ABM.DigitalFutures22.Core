using ABxM.Core.Agent;
using ABxM.Core.Behavior;
using ABxM.PrinciaplStrips.Core.Agent;
using ABxM.PrinciaplStrips.Core.AgentSystem;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace ABxM.PrincipalStrips.Core.Behaviors
{
    public class DescentUVBehavior : BehaviorBase
    {
        public double Stepsize;
        public bool Flip;
        //public Vector3d derivatives[1];

        public DescentUVBehavior(double _stepsize)
        {
            Stepsize = _stepsize;
            Weight = 1.0;
        }

        public override void Execute(AgentBase agent)
        {
            UmbilicalAgent myAgent = (UmbilicalAgent)agent;
            UmbilicalAgentSystem mySystem = (UmbilicalAgentSystem)myAgent.AgentSystem;

            // get the curvature at the current location
            SurfaceCurvature curvature = mySystem.SystemSurface.CurvatureAt(myAgent.UV.X, myAgent.UV.Y);
            double uCurvature = curvature.Kappa(0);
            double vCurvature = curvature.Kappa(1);
            double delta = Math.Abs(uCurvature - vCurvature);
            double tol = 0.000005;
            if (delta < tol)
            {
                //Rhino.RhinoApp.WriteLine("Agent " + myAgent.Id + ": I've arrived at an umbilical point!");
                myAgent.isFinished = true;
                return;
            }

            // let's assume we're at a point where principal curvatures are different
            // question: which way do we have to go to reduce the difference between principal curvatures?
            // we know the principal curvature directions and the curvature amounts
            // approach: let's take a small step in the four directions of principal curvature and see if the delta
            // is reduced. Then go in the direction of the largest decrease

            Vector3d uDirection = curvature.Direction(0);
            uDirection.Unitize();
            Vector3d vDirection = curvature.Direction(1);
            vDirection.Unitize();

            double Factor = Stepsize * delta * 1000;

            Vector3d moveVectorUup = Factor * uDirection;
            Vector3d moveVectorUdown = moveVectorUup * -1;
            Vector3d moveVectorVup = Factor * vDirection;
            Vector3d moveVectorVdown = moveVectorVup * -1;

            List<Vector3d> testVectors = new List<Vector3d> { moveVectorUup, moveVectorUdown, moveVectorVup, moveVectorVdown };
            List<SurfaceCurvature> testCurvatures = new List<SurfaceCurvature>();
            List<double> testDeltas = new List<double>();

            foreach (Vector3d testVector in testVectors)
            {
                Point3d testPoint = curvature.Point + testVector;
                double uTest, vTest = 0.0;
                mySystem.SystemSurface.ClosestPoint(testPoint, out uTest, out vTest);
                SurfaceCurvature testCurvature = mySystem.SystemSurface.CurvatureAt(uTest, vTest);
                testCurvatures.Add(testCurvature);
                double uKappa = testCurvature.Kappa(0);
                double vKappa = testCurvature.Kappa(1);
                double testDelta = Math.Abs(uKappa - vKappa);
                //double testDelta = Math.Abs(Math.Abs(uKappa) - Math.Abs(vKappa));
                testDeltas.Add(testDelta);
            }

            //now find the min testDelta
            Double minDelta = testDeltas[0];
            int minIndex = 0;
            for (int i = 1; i < testDeltas.Count; i++)
            {
                if (testDeltas[i] < minDelta)
                {
                    minDelta = testDeltas[i];
                    minIndex = i;
                }
            }

            if (minDelta < delta)
            {
                //now choose the direction and curvature that will result in the largest decrease
                Vector3d targetDirection = testVectors[minIndex];
                SurfaceCurvature targetCurvature = testCurvatures[minIndex];

                //get the UV parameters at the target location
                Point2d targetUVPoint = targetCurvature.UVPoint;

                Vector2d move2D = targetUVPoint - myAgent.UV;
                move2D.Unitize();
                move2D *= Factor;

                myAgent.Moves.Add(move2D);
                myAgent.Weights.Add(Weight);
            }
        }
    }

}
