using ABxM.Core.Agent;
using ABxM.Core.Behavior;
using ABxM.PrincipalStrips.Core.Agent;
using ABxM.PrincipalStrips.Core.AgentSystem;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace ABxM.PrincipalStrips.Core.Behaviors
{
    public class RailBehavior : BehaviorBase
    {
        public double Stepsize;
        public bool CurvatureDir;

        public int cases = 1;

        public double CurvatureAccuracy = 20;
        public int alg = 3;
        public RailBehavior(double _stepsize, bool _Dir, int icases)
        {
            Stepsize = _stepsize;
            CurvatureDir = _Dir;
            Weight = 1.0;
            cases = icases;
        }

        public override void Execute(AgentBase agent)
        {
            RailAgent myAgent = (RailAgent)agent;
            RailAgentSystem mySystem = (RailAgentSystem)myAgent.AgentSystem;

            //myAgent.FindGuideCurve();
            myAgent.FindSameGuideCurve();
            //myAgent.ComputeT(CurvatureAccuracy, CurvatureDir, 0.001, 3);
            myAgent.FindNeighbors();
            if (myAgent.isFixed != true)
            {

                /////////////////////////////////////////////////////
                Curve Cc = new PolylineCurve();


                //Declare our list of samples.
                List<Point3d> dir0 = myAgent.SampleCurvature(CurvatureAccuracy, CurvatureDir, 0, alg);
                List<Point3d> dir1 = myAgent.SampleCurvature(CurvatureAccuracy, CurvatureDir, Math.PI, alg);

                List<Point3d> outDir = new List<Point3d>();

                //test if the curvature line is CLOSED
                if (dir0[0] == dir0[dir0.Count - 1]) outDir = dir0;

                //Remove the first point in dir1 as it's a duplicate
                else
                {
                    dir1.RemoveAt(0);
                    dir1.Reverse();
                    dir1.AddRange(dir0);

                    outDir = dir1;
                }

                Cc = new PolylineCurve(outDir);
                Cc.Domain = new Interval(0.0, 1.0);
                //foreach(RailAgent neighborAgent in myAgent.Neighbors)


                //if (neighborAgent != null)
                if (myAgent.Neighbors[0] != null && myAgent.Neighbors[1] != null)
                {

                    //if (myAgent.Neighbors[0] == myAgent.Neighbors[1]) myAgent.isFixed = true;
                    //else
                    //{
                    RailAgent neighborAgent0 = myAgent.Neighbors[0];

                    Curve CNeighbor0 = new PolylineCurve();

                    //Declare our list of samples.
                    List<Point3d> dir2 = neighborAgent0.SampleCurvature(CurvatureAccuracy, CurvatureDir, 0, alg);
                    List<Point3d> dir3 = neighborAgent0.SampleCurvature(CurvatureAccuracy, CurvatureDir, Math.PI, alg);

                    List<Point3d> outDir1 = new List<Point3d>();

                    //test if the curvature line is CLOSED
                    if (dir2[0] == dir2[dir2.Count - 1]) outDir1 = dir2;

                    //Remove the first point in dir1 as it's a duplicate
                    else
                    {
                        dir3.RemoveAt(0);
                        dir3.Reverse();
                        dir3.AddRange(dir2);

                        outDir1 = dir3;
                    }

                    CNeighbor0 = new PolylineCurve(outDir1);
                    CNeighbor0.Domain = new Interval(0.0, 1.0);
                    /////////////////////
                    RailAgent neighborAgent1 = myAgent.Neighbors[1];

                    Curve CNeighbor1 = new PolylineCurve();

                    //Declare our list of samples.
                    List<Point3d> dir4 = neighborAgent1.SampleCurvature(CurvatureAccuracy, CurvatureDir, 0, alg);
                    List<Point3d> dir5 = neighborAgent1.SampleCurvature(CurvatureAccuracy, CurvatureDir, Math.PI, alg);

                    List<Point3d> outDir2 = new List<Point3d>();

                    //test if the curvature line is CLOSED
                    if (dir4[0] == dir4[dir4.Count - 1]) outDir2 = dir4;

                    //Remove the first point in dir1 as it's a duplicate
                    else
                    {
                        dir5.RemoveAt(0);
                        dir5.Reverse();
                        dir5.AddRange(dir4);

                        outDir2 = dir5;
                    }

                    CNeighbor1 = new PolylineCurve(outDir2);

                    CNeighbor1.Domain = new Interval(0.0, 1.0);

                    //////////////////////
                    List<double> distances0;
                    double[] curveDis0 = new double[2];
                    distances0 = myAgent.ComputeCurveDistance(Cc, CNeighbor0, out curveDis0);

                    List<double> distances1;
                    double[] curveDis1 = new double[2];
                    distances1 = myAgent.ComputeCurveDistance(Cc, CNeighbor1, out curveDis1);

                    ////////////////////////compute the move in two directions, compare the result distance, and find the distance-increase Direction.
                    double t1 = myAgent.t[0];
                    double t2 = neighborAgent1.t[0];
                    if (neighborAgent1.t.Length > 1)
                    {
                        if (Math.Abs(neighborAgent1.t[0] - t1) > Math.Abs(neighborAgent1.t[1] - myAgent.t[0]))
                        { t2 = neighborAgent1.t[1]; }
                    }
                    double u21, v21;
                    mySystem.SystemSurface.ClosestPoint(myAgent.GuideCurve.PointAt(t2), out u21, out v21);
                    Point2d p2d1 = new Point2d(u21, v21);

                    //double t1 = myAgent.t[0];
                    double t0 = neighborAgent0.t[0];
                    if (neighborAgent0.t.Length > 1)
                    {
                        if (Math.Abs(neighborAgent0.t[0] - t1) > Math.Abs(neighborAgent0.t[1] - myAgent.t[0]))
                        { t0 = neighborAgent0.t[1]; }
                    }
                    double u02, v20;
                    mySystem.SystemSurface.ClosestPoint(myAgent.GuideCurve.PointAt(t0), out u02, out v20);
                    Point2d p2d = new Point2d(u02, v20);

                    Vector2d DIR = p2d - myAgent.UV;

                    /////////////////////////////////////
                    double lengthGuideCurve = myAgent.GuideCurve.GetLength();
                    Vector2d move2D = new Vector2d();
                    double tRatio = double.NaN;

                    switch (cases)
                    {
                        //move according to Mean distance between curves(distances0; distances1;)
                        case 1:

                            //first neighbor curve
                            double msum = 0;//mean
                            double average;

                            for (int i = 0; i < distances0.Count; i++)
                            {
                                msum += distances0[i];
                            }
                            average = (double)(msum / distances0.Count);
                            /*double ssum = 0;//Standard Deviation
                            double std_dev;
                            for (int j = 0; j < distances0.Count; j++)
                            {
                                ssum += Math.Pow((distances0[j] - average), 2);
                            }
                            std_dev = Math.Sqrt(ssum / distances0.Count);*/



                            //second neighbor curve
                            double msum1 = 0;//mean
                            double average1;

                            for (int i = 0; i < distances1.Count; i++)
                            {
                                msum1 += distances1[i];
                            }
                            average1 = (double)(msum1 / distances1.Count);
                            /*double ssum1 = 0;//Standard Deviation
                            double std_dev1;
                            for (int j = 0; j < distances1.Count; j++)
                            {
                                ssum1 += Math.Pow((distances1[j] - average1), 2);
                            }
                            std_dev1 = Math.Sqrt(ssum1 / distances1.Count);*/


                            double dev1 = Stepsize * Math.Abs(average - average1);
                            tRatio = dev1 / lengthGuideCurve;
                            if (average < average1)
                            {
                                move2D = -1 * DIR;
                            }
                            else
                            {
                                move2D = DIR;
                            }

                            break;


                        //move according to Curvature value distribute case 2 的基础上加上曲率的权重
                        case 2:

                            //先求一下平均间距

                            double msums = 0;//mean
                            double averages;

                            for (int i = 0; i < distances0.Count; i++)
                            {
                                msums += distances0[i];
                            }
                            averages = (double)(msums / distances0.Count);


                            //second neighbor curve
                            double msums1 = 0;//mean
                            double averages1;

                            for (int i = 0; i < distances1.Count; i++)
                            {
                                msums1 += distances1[i];
                            }
                            averages1 = (double)(msums1 / distances1.Count);








                            int kappaPara = 1;
                            if (CurvatureDir) { kappaPara = 0; }

                            /*SurfaceCurvature crv = mySystem.SystemSurface.CurvatureAt(myAgent.UV.X, myAgent.UV.Y);
                            double curvatureThis= crv.Kappa(kappaPara);*/


                            double maxa = double.MinValue;

                            //多取n个中间点
                            double tmid0 = t1;
                            List<double> sumCurList = new List<double>();
                            double sumCur0 = 0.0;
                            double StepT = (t0 - t1) * (1.0 / 20.0);
                            for (int i = 0; i < 21; i++)
                            {
                                double ttt = tmid0 + (double)StepT * i;
                                double u, v;
                                mySystem.SystemSurface.ClosestPoint(myAgent.GuideCurve.PointAt(ttt), out u, out v);

                                SurfaceCurvature midcrv = mySystem.SystemSurface.CurvatureAt(u, v);
                                double curvatureMid = midcrv.Kappa(kappaPara);
                                sumCurList.Add(curvatureMid);
                            }

                            for (int i = 0; i < sumCurList.Count; i++)
                            {
                                //double disCur = Math.Abs(sumCurList[i + 1] - sumCurList[i]);
                                double disCur = Math.Abs(sumCurList[i]);

                                if (disCur > maxa) maxa = disCur;

                                sumCur0 += disCur;
                            }
                            sumCur0 = sumCur0 / 20;
                            //sumCur0 = sumCur0*Math.Abs(sumCurList[sumCurList.Count - 1]) * Math.Abs(sumCurList[sumCurList.Count - 1])*100;




                            /*SurfaceCurvature midcrv0 = mySystem.SystemSurface.CurvatureAt((myAgent.UV.X+neighborAgent0.UV.X)*0.5, (myAgent.UV.Y+ neighborAgent0.UV.Y)*0.5);
                            double curvatureMid0 = midcrv0.Kappa(kappaPara);

                            SurfaceCurvature crv0 = mySystem.SystemSurface.CurvatureAt(neighborAgent0.UV.X, neighborAgent0.UV.Y);
                            double curNeighborAgent0 = crv0.Kappa(kappaPara);*/

                            //多取n个中间点
                            double tmid2 = t1;
                            List<double> sumCurList1 = new List<double>();
                            double sumCur1 = 0.0;
                            double StepT1 = (t2 - t1) * (1.0 / 20.0);
                            for (int i = 0; i < 21; i++)
                            {
                                double ttt = tmid2 + (double)(StepT1 * i);
                                double u, v;
                                mySystem.SystemSurface.ClosestPoint(myAgent.GuideCurve.PointAt(ttt), out u, out v);

                                SurfaceCurvature midcrv = mySystem.SystemSurface.CurvatureAt(u, v);
                                double curvatureMid = midcrv.Kappa(kappaPara);
                                sumCurList1.Add(curvatureMid);
                            }

                            for (int i = 0; i < sumCurList1.Count; i++)
                            {
                                //double disCur = Math.Abs(sumCurList1[i + 1] - sumCurList1[i]);
                                double disCur = Math.Abs(sumCurList1[i]);

                                if (disCur > maxa) maxa = disCur;


                                sumCur1 += disCur;
                            }
                            sumCur1 = sumCur1 / 20;

                            sumCur0 = Math.Sqrt(Math.Sqrt(sumCur0 / maxa));
                            sumCur1 = Math.Sqrt(Math.Sqrt(sumCur1 / maxa));

                            //sumCur1 = sumCur1*Math.Abs(sumCurList1[sumCurList1.Count - 1]) * Math.Abs(sumCurList1[sumCurList1.Count - 1])*100;



                            //多取一个中间点
                            /*SurfaceCurvature midcrv1 = mySystem.SystemSurface.CurvatureAt((myAgent.UV.X + neighborAgent1.UV.X) * 0.5, (myAgent.UV.Y + neighborAgent1.UV.Y) * 0.5);
                            double curvatureMid1 = midcrv1.Kappa(kappaPara);

                            SurfaceCurvature crv1 = mySystem.SystemSurface.CurvatureAt(neighborAgent1.UV.X, neighborAgent1.UV.Y);
                            double curNeighborAgent1 = crv1.Kappa(kappaPara);*/

                            //double curvatureDis0 = Math.Abs(curNeighborAgent0- curvatureMid0) + Math.Abs(curvatureMid0-curvatureThis);
                            //double curvatureDis1 = Math.Abs(curNeighborAgent1 - curvatureMid1) + Math.Abs(curvatureMid1 - curvatureThis);




                            double dev2 = Stepsize * Math.Abs(sumCur1 * averages1 - sumCur0 * averages);

                            tRatio = dev2 / lengthGuideCurve;
                            if (sumCur0 * averages < sumCur1 * averages1)
                            {
                                move2D = -1 * DIR;
                            }
                            else
                            {
                                move2D = DIR;
                            }

                            break;

                            /*double dev2 = Stepsize * Math.Sqrt(Math.Abs(sumCur1 * curveDis1[1] - sumCur0 * curveDis0[1]));

                            tRatio = dev2*0.1;//dev2 / lengthGuideCurve;
                            if (Math.Sqrt(sumCur1 * curveDis1[1]) < Math.Sqrt(sumCur0 * sumCur0 * curveDis0[1]))
                            {
                                move2D =  DIR;
                            }
                            else
                            {
                                move2D =  -1 *DIR;
                            }

                            break;*/

                    }

                    move2D.Unitize();
                    move2D *= tRatio;

                    myAgent.Moves.Add(move2D);
                    myAgent.Weights.Add(Weight);
                }

            }

        }
    }

}
