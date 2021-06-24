/*
*  RVOAgent.cs
*  RVO2 Library.
*  
* Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
*/



using System;
using System.Collections.Generic;
using UnityEngine;

namespace RVO
{
    internal class RVOAgent : Agent
    {
        const float M_PI = 3.14159265358979323846f;
        const float RVO_EPSILON = 0.00001f;
        float sharing = 0.5f;

        /* New RVOAgent : 
         * Mur_A_Droite is an object used to know if there is a wall near by an agent, and on which side the wall is.
         * if Mur_A_Droite = 0 : there is no wall to interact with
         * if Mur_A_Droite = 1 : there is a wall on the right side of the agent
         * if Mur_A_Droite = -1 : there is a wall on the left side of the agent
         * 
         * This float is only to use in a specific context : pedestrians walking on a sidewalk, with possibly buildings on one side.
         * #Cleo */
        float Mur_A_Droite = 0f;

        public bool test_data_saving = false;
        public RVOAgent(RVOSimulator sim) : base(sim) { }

        /* Search for the best new velocity. */
        public override void computeNewVelocity()
        {
            orcaLines_.Clear();

            Mur_A_Droite = 0f; //Before checking for obstacles, we have to make sure to reinitialize this variable #Cleo
            for (int i = 0; i < obstacleNeighbors_.Count; ++i)
            {
                interactWith(obstacleNeighbors_[i].Value);
            }
            int numObstLines = orcaLines_.Count;

            /* Create neighbors ORCA lines. */
            interactWithAgents();

            bool neighboor = false;
            if (agentNeighbors_.Count == 0)
                neighboor = true;

            int lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, ref newVelocity_);

            if (lineFail < orcaLines_.Count)
            {
                linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, ref newVelocity_);
            }
        }

        public override void interactWith(Agent agent) //Initially a SuperAgent #Cleo
        {
            Agent other = agent; // Initially a SuperAgent #Cleo

            Vector2 relativePosition = other.position_ - position_;
            Vector2 relativeVelocity = velocity_ - other.velocity_;
            float distSq = (float)Math.Round(Vector2.absSq(relativePosition), 3);
            // Initial definition a the combinedRadius, modified to fit the new RVOAgent model #Cleo
            //float combinedRadius = radius_ + other.radius_; 

            /* New RVOAgent : 
             * When two agents have the same TimeHorizon and one of them have to stay on the sidewalk,
             * it may be stuck for a few seconds while the other agent is going down the sidewalk.
             * The solution to this problem would be to to temporarily change the first agent's radius, but only from its own point of view.
             * With a smaller radius, it does not get stuck because the presence of an other agent not far from it is not a problem anymore.
             * It may be compared to an acceptance of the agent to reduce its own space. 
             * The other agent still perceive the first agent with the same radius.
             * #Cleo */
            float temporaryRadius_ = radius_;
            if ((timeHorizon_ == other.timeHorizon_) && (Mur_A_Droite == 1)) { temporaryRadius_ = 0.4f; }
            float combinedRadius = temporaryRadius_ + other.radius_;


            float combinedRadiusSq = (float)Math.Round(Vector2.sqr(combinedRadius), 3);
            Vector2 w = new Vector2();
            Line line = new Line();
            Vector2 u;



            /* New RVOAgent : 
             * Regarding their TimeHorizon each agent take a different part in the effort made during the collision avoidance 
             * A higher TimeHorizon makes the agent take a greater part in the avoidance 
             * #Cleo */
            if (other.timeHorizon_ > timeHorizon_) { sharing = 0.3f; }
            if (other.timeHorizon_ < timeHorizon_) { sharing = 0.7f; }


            if (distSq > combinedRadiusSq)
            {
                /* No collision. */
                w = relativeVelocity - (1.0f / timeHorizon_) * relativePosition;

                /* Vector from cutoff center to relative velocity. */
                float wLengthSq = Vector2.absSq(w);
                Vector2 unitW = new Vector2();

                float dotProduct1 = w * relativePosition;
                if (dotProduct1 < 0.0f && Vector2.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                {
                    /* Project on cut-off circle. */
                    float wLength = (float)Math.Round(Math.Sqrt(wLengthSq), 3);
                    unitW = w / wLength;
                    line.direction = new Vector2(unitW.y(), -unitW.x());
                    u = (combinedRadius * (1.0f / timeHorizon_) - wLength) * unitW;
                }

                else
                {
                    test_data_saving = true;


                    //Project on legs.   
                    float leg = (float)Math.Round(Math.Sqrt(distSq - combinedRadiusSq), 3);


                    /* New RVOAgent : 
                     * Regarding the context, the agents may have to avoid each other by turning left. In the initial script, every collision avoidance manoeuvre is made by turning right.
                     * If an agent has a bigger part in the collision avoidance and has a wall on its right, it will not be able to avoid the collision with the other agent properly. This is why it has to turn left.
                     * Same thing if the agent has a smaller part in the collision avoidance and has a wall on its left. 
                     * When both the agents have the same involvement in the collision avoidance, the agents will turn right by convention 
                     * (this convention only applies in countries where cars drive on the right side of the road)
                     * #Cleo */
                    if (((sharing == 0.7f) && Mur_A_Droite == 1) || ((sharing == 0.3f) && (Mur_A_Droite == -1))) //If the agents have to turn left
                    {
                        if (Vector2.det(relativePosition, w) > 0.0f)  //new script
                        {
                            // Project on left leg. 
                            line.direction = new Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, (relativePosition.x() * combinedRadius + relativePosition.y() * leg)) / distSq;
                        }
                        else
                        {
                            // Project on right leg. 
                            line.direction = -new Vector2(-(relativePosition.x() * leg + relativePosition.y() * combinedRadius), -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                            //A minus sign has been added to the vector abscissa
                        }

                    }
                    else //If the agents have to turn right (convention), initial script
                    {

                        if (Vector2.det(relativePosition, w) > 0.0f)
                        {
                            // Project on left leg. 
                            line.direction = new Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                        }
                        else
                        {
                            // Project on right leg. 
                            line.direction = -new Vector2((relativePosition.x() * leg + relativePosition.y() * combinedRadius), -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                        }

                    }
                    float dotProduct2 = (float)Math.Round(relativeVelocity * line.direction, 3);
                    u = dotProduct2 * line.direction - relativeVelocity;
                }
            }
            else
            {
                /* Collision. Project on cut-off circle of time timeStep. */
                float invTimeStep = (float)Math.Round(1.0f / sim_.getTimeStep(), 3);

                /* Vector from cutoff center to relative velocity. */
                w = relativeVelocity - invTimeStep * relativePosition;

                float wLength = (float)Math.Round(Vector2.abs(w), 3);
                Vector2 unitW = w / wLength;

                line.direction = new Vector2(unitW.y(), -unitW.x());
                u = (combinedRadius * invTimeStep - wLength) * unitW;

            }

            /* This is where you can choose the proportion of responsabilities that each agents takes in avoiding collision 
             * sharing initially was set at 0.5f, in the original model both the agents have the same involvement in the collision avoidance
             * #Cleo */
            line.point = velocity_ + sharingEffort * u; //sharing
            orcaLines_.Add(line);
        }


        public override void interactWith(Obstacle obstacle)

        {

            Obstacle obstacle1 = obstacle;
            Obstacle obstacle2 = obstacle1.next_;


            Vector2 relativePosition1 = obstacle1.point_ - position_;
            Vector2 relativePosition2 = obstacle2.point_ - position_;


            /* New RVOAgent :
             * In the specific case of two agents walking on a side walk, the obstacles the agents have to interact with are the buildings (perceived as walls on the side)
             * To know where these walls are (right or left side of the agents), we use two vectors : 
             * - The oriented Vector between the agent's position and the obstacle's position (relativePosition1)
             * - The oriented Vector between the agent's position and its goal's position (AG, good indicator of the preferred direction of the agent)
             * By using a cross product, we can know where is the wall from the agent's point of view (left or right)
             * #Cleo */
            Vector2 AG = goal_ - position_;

            float cp = AG.x_ * relativePosition1.y_ - AG.y_ * relativePosition1.x_; // Computes the position of the wall relative to the agent
            if (cp < 0) { Mur_A_Droite = 1; }
            if (cp > 0) { Mur_A_Droite = -1; }


            /* 
            * Check if velocity obstacle of obstacle is already taken care of by
            * previously constructed obstacle ORCA lines.
            */
            bool alreadyCovered = false;

            for (int j = 0; j < orcaLines_.Count; ++j)
            {
                if (Vector2.det((1.0f / timeHorizonObst_) * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - (1.0f / timeHorizonObst_) * radius_ >= -RVO_EPSILON
                    && Vector2.det((1.0f / timeHorizonObst_) * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - (1.0f / timeHorizonObst_) * radius_ >= -RVO_EPSILON)
                {
                    alreadyCovered = true;
                    break;
                }
            }

            if (alreadyCovered)
            {
                return;
            }

            /* Not yet covered. Check for collisions. */

            float distSq1 = Vector2.absSq(relativePosition1);
            float distSq2 = Vector2.absSq(relativePosition2);

            float radiusSq = Vector2.sqr(radius_);

            Vector2 obstacleVector = obstacle2.point_ - obstacle1.point_;
            float s = (-relativePosition1 * obstacleVector) / Vector2.absSq(obstacleVector);
            float distSqLine = Vector2.absSq(-relativePosition1 - s * obstacleVector);

            Line line;

            if (s < 0 && distSq1 <= radiusSq)
            {
                /* Collision with left vertex. Ignore if non-convex. */
                if (obstacle1.convex_)
                {
                    line.point = new Vector2(0, 0);
                    line.direction = Vector2.normalize(new Vector2(-relativePosition1.y(), relativePosition1.x()));
                    orcaLines_.Add(line);
                }
                return;
            }
            else if (s > 1 && distSq2 <= radiusSq)
            {
                /* Collision with right vertex. Ignore if non-convex 
                * or if it will be taken care of by neighoring obstace */
                if (obstacle2.convex_ && Vector2.det(relativePosition2, obstacle2.direction_) >= 0)
                {
                    line.point = new Vector2(0, 0);
                    line.direction = Vector2.normalize(new Vector2(-relativePosition2.y(), relativePosition2.x()));
                    orcaLines_.Add(line);
                }
                return;
            }
            else if (s >= 0 && s < 1 && distSqLine <= radiusSq)
            {
                /* Collision with obstacle segment. */
                line.point = new Vector2(0, 0);
                line.direction = -obstacle1.direction_;
                orcaLines_.Add(line);
                return;
            }

            /* 
            * No collision.  
            * Compute legs. When obliquely viewed, both legs can come from a single
            * vertex. Legs extend cut-off line when nonconvex vertex.
            */

            Vector2 leftLegDirection, rightLegDirection;

            if (s < 0 && distSqLine <= radiusSq)
            {
                /*
                * Obstacle viewed obliquely so that left vertex
                * defines velocity obstacle.
                */
                if (!obstacle1.convex_)
                {
                    /* Ignore obstacle. */
                    return;
                }

                obstacle2 = obstacle1;

                float leg1 = (float)Math.Sqrt(distSq1 - radiusSq);
                leftLegDirection = new Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
                rightLegDirection = new Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
            }
            else if (s > 1 && distSqLine <= radiusSq)
            {
                /*
                * Obstacle viewed obliquely so that
                * right vertex defines velocity obstacle.
                */
                if (!obstacle2.convex_)
                {
                    /* Ignore obstacle. */
                    return;
                }

                obstacle1 = obstacle2;

                float leg2 = (float)Math.Sqrt(distSq2 - radiusSq);
                leftLegDirection = new Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
                rightLegDirection = new Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
            }
            else
            {
                /* Usual situation. */
                if (obstacle1.convex_)
                {
                    float leg1 = (float)Math.Sqrt(distSq1 - radiusSq);
                    leftLegDirection = new Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
                }
                else
                {
                    /* Left vertex non-convex; left leg extends cut-off line. */
                    leftLegDirection = -obstacle1.direction_;
                }

                if (obstacle2.convex_)
                {
                    float leg2 = (float)Math.Sqrt(distSq2 - radiusSq);
                    rightLegDirection = new Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
                }
                else
                {
                    /* Right vertex non-convex; right leg extends cut-off line. */
                    rightLegDirection = obstacle1.direction_;
                }
            }

            /* 
            * Legs can never point into neighboring edge when convex vertex,
            * take cutoff-line of neighboring edge instead. If velocity projected on
            * "foreign" leg, no constraint is added. 
            */

            Obstacle leftNeighbor = obstacle1.previous_;

            bool isLeftLegForeign = false;
            bool isRightLegForeign = false;


            if (obstacle.convex_ && Vector2.det(leftLegDirection, -leftNeighbor.direction_) >= 0.0f)
            {
                /* Left leg points into obstacle. */
                leftLegDirection = -leftNeighbor.direction_;
                isLeftLegForeign = true;
            }

            if (obstacle2.convex_ && Vector2.det(rightLegDirection, obstacle2.direction_) <= 0.0f)
            {
                /* Right leg points into obstacle. */
                rightLegDirection = obstacle2.direction_;
                isRightLegForeign = true;
            }

            /* Compute cut-off centers. */
            Vector2 leftCutoff = (1.0f / timeHorizonObst_) * (obstacle1.point_ - position_);
            Vector2 rightCutoff = (1.0f / timeHorizonObst_) * (obstacle2.point_ - position_);
            Vector2 cutoffVec = rightCutoff - leftCutoff;

            /* Project current velocity on velocity obstacle. */

            /* Check if current velocity is projected on cutoff circles. */
            float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutoff) * cutoffVec) / Vector2.absSq(cutoffVec));
            float tLeft = ((velocity_ - leftCutoff) * leftLegDirection);
            float tRight = ((velocity_ - rightCutoff) * rightLegDirection);

            if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f))
            {
                /* Project on left cut-off circle. */
                Vector2 unitW = Vector2.normalize(velocity_ - leftCutoff);

                line.direction = new Vector2(unitW.y(), -unitW.x());
                line.point = leftCutoff + radius_ * (1.0f / timeHorizonObst_) * unitW;
                orcaLines_.Add(line);
                return;
            }
            else if (t > 1.0f && tRight < 0.0f)
            {
                /* Project on right cut-off circle. */
                Vector2 unitW = Vector2.normalize(velocity_ - rightCutoff);

                line.direction = new Vector2(unitW.y(), -unitW.x());
                line.point = rightCutoff + radius_ * (1.0f / timeHorizonObst_) * unitW;
                orcaLines_.Add(line);
                return;
            }

            /* 
            * Project on left leg, right leg, or cut-off line, whichever is closest
            * to velocity.
            */
            float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? Single.PositiveInfinity : Vector2.absSq(velocity_ - (leftCutoff + t * cutoffVec)));
            float distSqLeft = ((tLeft < 0.0f) ? Single.PositiveInfinity : Vector2.absSq(velocity_ - (leftCutoff + tLeft * leftLegDirection)));
            float distSqRight = ((tRight < 0.0f) ? Single.PositiveInfinity : Vector2.absSq(velocity_ - (rightCutoff + tRight * rightLegDirection)));

            if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
            {
                /* Project on cut-off line. */
                line.direction = -obstacle1.direction_;
                line.point = leftCutoff + radius_ * (1.0f / timeHorizonObst_) * new Vector2(-line.direction.y(), line.direction.x());
                orcaLines_.Add(line);
                return;
            }
            else if (distSqLeft <= distSqRight)
            {
                /* Project on left leg. */
                if (isLeftLegForeign)
                {
                    return;
                }

                line.direction = leftLegDirection;
                line.point = leftCutoff + radius_ * (1.0f / timeHorizonObst_) * new Vector2(-line.direction.y(), line.direction.x());
                orcaLines_.Add(line);
                return;
            }
            else
            {
                /* Project on right leg. */
                if (isRightLegForeign)
                {
                    return;
                }

                line.direction = -rightLegDirection;
                line.point = rightCutoff + radius_ * (1.0f / timeHorizonObst_) * new Vector2(-line.direction.y(), line.direction.x());
                orcaLines_.Add(line);
                return;
            }
        }

        new internal void update()
        {
            velocityBuffer_.Add(velocity_);
            if (!Double.IsNaN(newVelocity_.x_) && !Double.IsNaN(newVelocity_.y_))
            {
                velocity_ = newVelocity_;
                position_ += velocity_ * sim_.timeStep_;
                cpt_dist += Vector2.abs(velocity_) * sim_.getTimeStep();
                cpt_time++;
            }
            else
            {
                newVelocity_ = new Vector2(0, 0);
                velocity_ = newVelocity_;
                position_ += velocity_ * sim_.timeStep_;
                cpt_dist += Vector2.abs(velocity_) * sim_.getTimeStep();
                cpt_time++;
            }
        }

        new public bool linearProgram1(IList<Line> lines, int lineNo, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            float dotProduct = lines[lineNo].point * lines[lineNo].direction;
            float discriminant = Vector2.sqr(dotProduct) + Vector2.sqr(radius) - Vector2.absSq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = (float)Math.Sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                float denominator = Vector2.det(lines[lineNo].direction, lines[i].direction);
                float numerator = Vector2.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (Math.Abs(denominator) <= RVO_EPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }
                    else
                    {
                        continue;
                    }
                }


                float t = numerator / denominator;
                if (denominator >= 0.0f)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }
            if (!directionOpt)
            {
                /* Optimize direction. */
                /* New RVOAgent :
                 * when two agents meet on a sidewalk, we are dealing with a particular case : the orcaline due to the other agent's presence is perpendicular to the agent's direction.
                 * In this particular case, in the original script, the agent was forced to turn right.
                 * But there are some cases where the agent should turn left to avoid the other agent, juste like line 166 of this script.
                 * With the same conditions we consider the cases where the agent have to take left.
                 * 
                 * The particular case appears when optVelocity * lines[lineNo].direction = 0.0f
                 * #Cleo */

                /*To avoid each other, the agents have to turn left*/
                if (((sharing == 0.7f) && Mur_A_Droite == 1) || ((sharing == 0.3f) && (Mur_A_Droite == -1)))
                {
                    if (optVelocity * lines[lineNo].direction >= 0.0f) //wide inequality
                    {
                        // Take right extreme. 
                        result = lines[lineNo].point + tRight * lines[lineNo].direction;
                    }
                    else
                    {
                        // Take left extreme. 
                        result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                    }
                }
                /*To avoid each other, the agents have to turn right*/
                else
                {
                    if (optVelocity * lines[lineNo].direction > 0.0f) //strict inequality
                    {
                        // Take right extreme. 
                        result = lines[lineNo].point + tRight * lines[lineNo].direction;
                    }
                    else
                    {
                        // Take left extreme. 
                        result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                    }
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }
            return true;
        }

        new int linearProgram2(IList<Line> lines, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            if (!directionOpt)
            {
                /* 
                * Optimize direction. Note that the optimization velocity is of unit
                * length in this case.
                */
                result = optVelocity * radius;
            }
            else if (Vector2.absSq(optVelocity) > Vector2.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = Vector2.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }



            for (int i = 0; i < lines.Count; ++i)
            {
                if (Vector2.det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector2 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;
                        return i;
                    }
                }

            }

            return lines.Count;
        }

        new void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, float radius, ref Vector2 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if ((float)Math.Round(Vector2.det(lines[i].direction, lines[i].point - result), 3) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Line> projLines = new List<Line>();
                    for (int k = 0; k < numObstLines; ++k) { projLines.Add(lines[k]); }


                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = Vector2.det(lines[i].direction, lines[j].direction);

                        if (Math.Abs(determinant) <= RVO_EPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (lines[i].direction * lines[j].direction > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = 0.5f * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (Vector2.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = Vector2.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    Vector2 tempResult = result;
                    if (linearProgram2(projLines, radius, new Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, ref result) < projLines.Count)
                    {
                        /* This should in principle not happen.  The result is by definition
                        * already in the feasible region of this linear program. If it fails,
                        * it is due to small floating point error, and the current result is
                        * kept.
                        */
                        result = tempResult;
                    }

                    distance = Vector2.det(lines[i].direction, lines[i].point - result);
                }
            }
        }

    }

}