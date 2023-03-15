using ABxM.Core.Agent;
using ABxM.Core.Behavior;
using ABxM.PrincipalStrips.Core.AgentSystem;
using Rhino.Geometry;
using System.Collections.Generic;

namespace ABxM.PrincipalStrips.Core.Agent
{
    public class UmbilicalAgent : AgentBase
    {
        public Point2d UV = Point2d.Unset;
        private Point2d startUV;
        public Polyline Trail = new Polyline();
        public bool isFinished = false;

        /// <summary>
        /// The list of 2-dimensional moves
        /// </summary>
        public List<Vector2d> Moves = new List<Vector2d>();
        public List<double> Weights = new List<double>();

        public UmbilicalAgent(Point2d uvParameter, List<BehaviorBase> behaviors)
        {
            this.UV = this.startUV = uvParameter;
            this.Behaviors = behaviors;
        }

        public override void Reset()
        {
            this.Moves.Clear();
            this.UV = this.startUV;
            this.Trail.Clear();
            this.isFinished = false;

        }

        public override void PreExecute()
        {
            this.Moves.Clear();
        }

        public override void Execute()
        {
            if (this.isFinished)
                return;

            foreach (BehaviorBase behavior in this.Behaviors)
                behavior.Execute(this);
        }

        public override void PostExecute()
        {
            if (this.Moves.Count > 0)
            {
                Vector2d totalWeightedMove = Vector2d.Zero;
                double totalWeight = 0.0;

                for (int i = 0; i < Moves.Count; ++i)
                {
                    totalWeightedMove += Weights[i] * Moves[i];
                    totalWeight += Weights[i];
                }
                this.UV += totalWeightedMove / totalWeight;
            }

            this.Trail.Add((this.AgentSystem as UmbilicalAgentSystem).SystemSurface.PointAt(UV.X, UV.Y));
            if (Trail.Count > 20)
                Trail.RemoveAt(0);
        }

        public override List<object> GetDisplayGeometries()
        {
            List<object> displayGeometry = new List<object>();

            //if ((this.AgentSystem as UVAgentSystem).SystemSurface == null)
            //    return displayGeometry;

            displayGeometry.Add((this.AgentSystem as UmbilicalAgentSystem).SystemSurface.PointAt(UV.X, UV.Y));

            if ((this.Trail.Count < 2))
                return displayGeometry;

            displayGeometry.Add(this.Trail);

            return displayGeometry;
        }


    }
}
