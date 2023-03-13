using ABxM.Core.AgentSystem;
using Grasshopper.Kernel.Geometry;
using Grasshopper.Kernel.Geometry.Delaunay;
using ICD.ABM.DigitalFutures22.Core.Agent;
using ICD.ABM.DigitalFutures22.Core.Environments;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;

namespace ICD.ABM.DigitalFutures22.Core.AgentSystem
{
    public partial class PanelAgentSystem : AgentSystemBase
    { 
        public DisplayPipeline Dpl;

        //public void DisplayMeshes(DisplayPipeline dpl, PanelAgentSystem system)
        //{
        //    Dpl = dpl;
        //}

        public void DisplayWires(DisplayPipeline dpl, PanelAgentSystem system)
        {
            Dpl = dpl;

            foreach (PanelAgent agent in system.Agents)
            {
                Dpl.DrawPoint(
                    agent.Position,
                    PointStyle.RoundSimple,
                    Color.Crimson,
                    Color.Crimson,
                    5f, 1f, 0f, 0f, true, true);

                foreach (LineCurve c in agent.DirectNeighborConnections)
                {
                    Dpl.DrawCurve(c, Color.DarkSeaGreen);
                }

                foreach (Curve c in agent.PanelCuts)
                {
                    Dpl.DrawCurve(c, Color.SteelBlue, 3);
                }
            }
        }
    }
}