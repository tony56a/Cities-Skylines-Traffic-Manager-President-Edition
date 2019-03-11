using ColossalFramework;
using ColossalFramework.Math;
using ColossalFramework.UI;
using System;
using System.Threading;
using UnityEngine;

namespace TrafficManager.Custom.PathFinding {
    public class StockPathFind2 : MonoBehaviour {
        private struct BufferItem {
            public PathUnit.Position m_position;
            public float m_comparisonValue;
            public float m_methodDistance;
            public float m_duration;
            public uint m_laneID;
            public NetInfo.Direction m_direction;
            public NetInfo.LaneType m_lanesUsed;
        }
        private const float BYTE_TO_FLOAT_OFFSET_CONVERSION_FACTOR = 0.003921569f;
        private const float TICKET_COST_CONVERSION_FACTOR = 3.92156863E-07f;
        private Array32<PathUnit> m_pathUnits;
        private uint m_queueFirst;
        private uint m_queueLast;
        private uint m_calculating;
        private object m_queueLock;
        private Thread m_pathFindThread;
        private bool m_terminated;
        public ThreadProfiler m_pathfindProfiler;
        public volatile int m_queuedPathFindCount;
        private object m_bufferLock;
        private int m_bufferMinPos;
        private int m_bufferMaxPos;
        private uint[] m_laneLocation;
        private PathUnit.Position[] m_laneTarget;
        private BufferItem[] m_buffer;
        private int[] m_bufferMin;
        private int[] m_bufferMax;
        private float m_maxLength;
        private uint m_startLaneA;
        private uint m_startLaneB;
        private uint m_endLaneA;
        private uint m_endLaneB;
        private uint m_vehicleLane;
        private byte m_startOffsetA;
        private byte m_startOffsetB;
        private byte m_vehicleOffset;
        private NetSegment.Flags m_carBanMask;
        private bool m_ignoreBlocked;
        private bool m_stablePath;
        private bool m_randomParking;
        private bool m_transportVehicle;
        private bool m_ignoreCost;
        private NetSegment.Flags m_disableMask;
        private Randomizer m_pathRandomizer;
        private uint m_pathFindIndex;
        private NetInfo.LaneType m_laneTypes;
        private VehicleInfo.VehicleType m_vehicleTypes;

        public bool IsAvailable {
            get { return m_pathFindThread.IsAlive; }
        }

        private void Awake() {
            m_pathfindProfiler = new ThreadProfiler();
            m_laneLocation = new uint[262144];
            m_laneTarget = new PathUnit.Position[262144];
            m_buffer = new BufferItem[65536];
            m_bufferMin = new int[1024];
            m_bufferMax = new int[1024];
            m_queueLock = new object();
            m_bufferLock = Singleton<PathManager>.instance.m_bufferLock;
            m_pathUnits = Singleton<PathManager>.instance.m_pathUnits;
            m_pathFindThread = new Thread(PathFindThread);
            m_pathFindThread.Name = "Pathfind";
            m_pathFindThread.Priority = SimulationManager.SIMULATION_PRIORITY;
            m_pathFindThread.Start();
            if (!m_pathFindThread.IsAlive) {
                CODebugBase<LogChannel>.Error(LogChannel.Core, "Path find thread failed to start!");
            }
        }

        private void OnDestroy() {
            while (!Monitor.TryEnter(m_queueLock, SimulationManager.SYNCHRONIZE_TIMEOUT)) {
            }

            try {
                m_terminated = true;
                Monitor.PulseAll(m_queueLock);
            } finally {
                Monitor.Exit(m_queueLock);
            }
        }

        public bool CalculatePath(uint unit, bool skipQueue) {
            if (Singleton<PathManager>.instance.AddPathReference(unit)) {
                while (!Monitor.TryEnter(m_queueLock, SimulationManager.SYNCHRONIZE_TIMEOUT)) {
                }

                try {
                    if (skipQueue) {
                        if (m_queueLast == 0) {
                            m_queueLast = unit;
                        } else {
                            m_pathUnits.m_buffer[unit].m_nextPathUnit = m_queueFirst;
                        }

                        m_queueFirst = unit;
                    } else {
                        if (m_queueLast == 0) {
                            m_queueFirst = unit;
                        } else {
                            m_pathUnits.m_buffer[m_queueLast].m_nextPathUnit = unit;
                        }

                        m_queueLast = unit;
                    }

                    m_pathUnits.m_buffer[unit].m_pathFindFlags |= 1;
                    m_queuedPathFindCount++;
                    Monitor.Pulse(m_queueLock);
                } finally {
                    Monitor.Exit(m_queueLock);
                }

                return true;
            }

            return false;
        }

        public void WaitForAllPaths() {
            while (!Monitor.TryEnter(m_queueLock, SimulationManager.SYNCHRONIZE_TIMEOUT)) {
            }

            try {
                while (true) {
                    if (m_queueFirst == 0 && m_calculating == 0) {
                        break;
                    }

                    if (m_terminated) {
                        break;
                    }

                    Monitor.Wait(m_queueLock);
                }
            } finally {
                Monitor.Exit(m_queueLock);
            }
        }

        private void PathFindImplementation(uint unit, ref PathUnit data) {
            NetManager instance = Singleton<NetManager>.instance;
            m_laneTypes = (NetInfo.LaneType) m_pathUnits.m_buffer[unit].m_laneTypes;
            m_vehicleTypes = (VehicleInfo.VehicleType) m_pathUnits.m_buffer[unit].m_vehicleTypes;
            m_maxLength = m_pathUnits.m_buffer[unit].m_length;
            m_pathFindIndex = (m_pathFindIndex + 1 & 0x7FFF);
            m_pathRandomizer = new Randomizer(unit);
            m_carBanMask = NetSegment.Flags.CarBan;
            if ((m_pathUnits.m_buffer[unit].m_simulationFlags & 0x10) != 0) {
                m_carBanMask |= NetSegment.Flags.HeavyBan;
            }

            if ((m_pathUnits.m_buffer[unit].m_simulationFlags & 4) != 0) {
                m_carBanMask |= NetSegment.Flags.WaitingPath;
            }

            m_ignoreBlocked = ((m_pathUnits.m_buffer[unit].m_simulationFlags & 0x20) != 0);
            m_stablePath = ((m_pathUnits.m_buffer[unit].m_simulationFlags & 0x40) != 0);
            m_randomParking = ((m_pathUnits.m_buffer[unit].m_simulationFlags & 0x80) != 0);
            m_transportVehicle = ((m_laneTypes & NetInfo.LaneType.TransportVehicle) != NetInfo.LaneType.None);
            m_ignoreCost = (m_stablePath || (m_pathUnits.m_buffer[unit].m_simulationFlags & 8) != 0);
            m_disableMask = (NetSegment.Flags.Collapsed | NetSegment.Flags.PathFailed);
            if ((m_pathUnits.m_buffer[unit].m_simulationFlags & 2) == 0) {
                m_disableMask |= NetSegment.Flags.Flooded;
            }

            if ((m_laneTypes & NetInfo.LaneType.Vehicle) != 0) {
                m_laneTypes |= NetInfo.LaneType.TransportVehicle;
            }

            int num = m_pathUnits.m_buffer[unit].m_positionCount & 0xF;
            int num2 = m_pathUnits.m_buffer[unit].m_positionCount >> 4;
            BufferItem bufferItem = default(BufferItem);
            if (data.m_position00.m_segment != 0 && num >= 1) {
                m_startLaneA = PathManager.GetLaneID(data.m_position00);
                m_startOffsetA = data.m_position00.m_offset;
                bufferItem.m_laneID = m_startLaneA;
                bufferItem.m_position = data.m_position00;
                GetLaneDirection(data.m_position00, out bufferItem.m_direction, out bufferItem.m_lanesUsed);
                bufferItem.m_comparisonValue = 0f;
                bufferItem.m_duration = 0f;
            } else {
                m_startLaneA = 0u;
                m_startOffsetA = 0;
            }

            BufferItem bufferItem2 = default(BufferItem);
            if (data.m_position02.m_segment != 0 && num >= 3) {
                m_startLaneB = PathManager.GetLaneID(data.m_position02);
                m_startOffsetB = data.m_position02.m_offset;
                bufferItem2.m_laneID = m_startLaneB;
                bufferItem2.m_position = data.m_position02;
                GetLaneDirection(data.m_position02, out bufferItem2.m_direction, out bufferItem2.m_lanesUsed);
                bufferItem2.m_comparisonValue = 0f;
                bufferItem2.m_duration = 0f;
            } else {
                m_startLaneB = 0u;
                m_startOffsetB = 0;
            }

            BufferItem bufferItem3 = default(BufferItem);
            if (data.m_position01.m_segment != 0 && num >= 2) {
                m_endLaneA = PathManager.GetLaneID(data.m_position01);
                bufferItem3.m_laneID = m_endLaneA;
                bufferItem3.m_position = data.m_position01;
                GetLaneDirection(data.m_position01, out bufferItem3.m_direction, out bufferItem3.m_lanesUsed);
                bufferItem3.m_methodDistance = 0.01f;
                bufferItem3.m_comparisonValue = 0f;
                bufferItem3.m_duration = 0f;
            } else {
                m_endLaneA = 0u;
            }

            BufferItem bufferItem4 = default(BufferItem);
            if (data.m_position03.m_segment != 0 && num >= 4) {
                m_endLaneB = PathManager.GetLaneID(data.m_position03);
                bufferItem4.m_laneID = m_endLaneB;
                bufferItem4.m_position = data.m_position03;
                GetLaneDirection(data.m_position03, out bufferItem4.m_direction, out bufferItem4.m_lanesUsed);
                bufferItem4.m_methodDistance = 0.01f;
                bufferItem4.m_comparisonValue = 0f;
                bufferItem4.m_duration = 0f;
            } else {
                m_endLaneB = 0u;
            }

            if (data.m_position11.m_segment != 0 && num2 >= 1) {
                m_vehicleLane = PathManager.GetLaneID(data.m_position11);
                m_vehicleOffset = data.m_position11.m_offset;
            } else {
                m_vehicleLane = 0u;
                m_vehicleOffset = 0;
            }

            BufferItem bufferItem5 = default(BufferItem);
            byte b = 0;
            m_bufferMinPos = 0;
            m_bufferMaxPos = -1;
            if (m_pathFindIndex == 0) {
                uint num3 = 4294901760u;
                for (int i = 0; i < 262144; i++) {
                    m_laneLocation[i] = num3;
                }
            }

            for (int j = 0; j < 1024; j++) {
                m_bufferMin[j] = 0;
                m_bufferMax[j] = -1;
            }

            if (bufferItem3.m_position.m_segment != 0) {
                m_bufferMax[0]++;
                m_buffer[++m_bufferMaxPos] = bufferItem3;
            }

            if (bufferItem4.m_position.m_segment != 0) {
                m_bufferMax[0]++;
                m_buffer[++m_bufferMaxPos] = bufferItem4;
            }

            bool flag = false;
            while (m_bufferMinPos <= m_bufferMaxPos) {
                int num4 = m_bufferMin[m_bufferMinPos];
                int num5 = m_bufferMax[m_bufferMinPos];
                if (num4 > num5) {
                    m_bufferMinPos++;
                } else {
                    m_bufferMin[m_bufferMinPos] = num4 + 1;
                    BufferItem bufferItem6 = m_buffer[(m_bufferMinPos << 6) + num4];
                    if (bufferItem6.m_position.m_segment == bufferItem.m_position.m_segment && bufferItem6.m_position.m_lane == bufferItem.m_position.m_lane) {
                        if ((bufferItem6.m_direction & NetInfo.Direction.Forward) != 0 && bufferItem6.m_position.m_offset >= m_startOffsetA) {
                            bufferItem5 = bufferItem6;
                            b = m_startOffsetA;
                            flag = true;
                            break;
                        }

                        if ((bufferItem6.m_direction & NetInfo.Direction.Backward) != 0 && bufferItem6.m_position.m_offset <= m_startOffsetA) {
                            bufferItem5 = bufferItem6;
                            b = m_startOffsetA;
                            flag = true;
                            break;
                        }
                    }

                    if (bufferItem6.m_position.m_segment == bufferItem2.m_position.m_segment && bufferItem6.m_position.m_lane == bufferItem2.m_position.m_lane) {
                        if ((bufferItem6.m_direction & NetInfo.Direction.Forward) != 0 && bufferItem6.m_position.m_offset >= m_startOffsetB) {
                            bufferItem5 = bufferItem6;
                            b = m_startOffsetB;
                            flag = true;
                            break;
                        }

                        if ((bufferItem6.m_direction & NetInfo.Direction.Backward) != 0 && bufferItem6.m_position.m_offset <= m_startOffsetB) {
                            bufferItem5 = bufferItem6;
                            b = m_startOffsetB;
                            flag = true;
                            break;
                        }
                    }

                    if ((bufferItem6.m_direction & NetInfo.Direction.Forward) != 0) {
                        ushort startNode = instance.m_segments.m_buffer[bufferItem6.m_position.m_segment].m_startNode;
                        ProcessItemMain(bufferItem6, startNode, ref instance.m_nodes.m_buffer[startNode], (byte) 0, false);
                    }

                    if ((bufferItem6.m_direction & NetInfo.Direction.Backward) != 0) {
                        ushort endNode = instance.m_segments.m_buffer[bufferItem6.m_position.m_segment].m_endNode;
                        ProcessItemMain(bufferItem6, endNode, ref instance.m_nodes.m_buffer[endNode], (byte) 255, false);
                    }

                    int num6 = 0;
                    ushort num7 = instance.m_lanes.m_buffer[bufferItem6.m_laneID].m_nodes;
                    if (num7 != 0) {
                        ushort startNode2 = instance.m_segments.m_buffer[bufferItem6.m_position.m_segment].m_startNode;
                        ushort endNode2 = instance.m_segments.m_buffer[bufferItem6.m_position.m_segment].m_endNode;
                        bool flag2 = ((instance.m_nodes.m_buffer[startNode2].m_flags | instance.m_nodes.m_buffer[endNode2].m_flags) & NetNode.Flags.Disabled) != NetNode.Flags.None;
                        while (num7 != 0) {
                            NetInfo.Direction direction = NetInfo.Direction.None;
                            byte laneOffset = instance.m_nodes.m_buffer[num7].m_laneOffset;
                            if (laneOffset <= bufferItem6.m_position.m_offset) {
                                direction |= NetInfo.Direction.Forward;
                            }

                            if (laneOffset >= bufferItem6.m_position.m_offset) {
                                direction |= NetInfo.Direction.Backward;
                            }

                            if ((bufferItem6.m_direction & direction) != 0 && (!flag2 || (instance.m_nodes.m_buffer[num7].m_flags & NetNode.Flags.Disabled) != 0)) {
                                ProcessItemMain(bufferItem6, num7, ref instance.m_nodes.m_buffer[num7], laneOffset, true);
                            }

                            num7 = instance.m_nodes.m_buffer[num7].m_nextLaneNode;
                            if (++num6 == 32768) {
                                break;
                            }
                        }
                    }
                }
            }

            if (!flag) {
                m_pathUnits.m_buffer[unit].m_pathFindFlags |= 8;
            } else {
                float num8 = (m_laneTypes != NetInfo.LaneType.Pedestrian && (m_laneTypes & NetInfo.LaneType.Pedestrian) != 0) ? bufferItem5.m_duration : bufferItem5.m_methodDistance;
                m_pathUnits.m_buffer[unit].m_length = num8;
                m_pathUnits.m_buffer[unit].m_speed = (byte) Mathf.Clamp(bufferItem5.m_methodDistance * 100f / Mathf.Max(0.01f, bufferItem5.m_duration), 0f, 255f);
                uint num9 = unit;
                int num10 = 0;
                int num11 = 0;
                PathUnit.Position position = bufferItem5.m_position;
                if ((position.m_segment != bufferItem3.m_position.m_segment || position.m_lane != bufferItem3.m_position.m_lane || position.m_offset != bufferItem3.m_position.m_offset) &&
                    (position.m_segment != bufferItem4.m_position.m_segment || position.m_lane != bufferItem4.m_position.m_lane || position.m_offset != bufferItem4.m_position.m_offset)) {
                    if (b != position.m_offset) {
                        PathUnit.Position position2 = position;
                        position2.m_offset = b;
                        m_pathUnits.m_buffer[num9].SetPosition(num10++, position2);
                    }

                    m_pathUnits.m_buffer[num9].SetPosition(num10++, position);
                    position = m_laneTarget[bufferItem5.m_laneID];
                }

                for (int k = 0; k < 262144; k++) {
                    m_pathUnits.m_buffer[num9].SetPosition(num10++, position);
                    if (position.m_segment == bufferItem3.m_position.m_segment && position.m_lane == bufferItem3.m_position.m_lane && position.m_offset == bufferItem3.m_position.m_offset) {
                        goto IL_0cad;
                    }

                    if (position.m_segment == bufferItem4.m_position.m_segment && position.m_lane == bufferItem4.m_position.m_lane && position.m_offset == bufferItem4.m_position.m_offset) {
                        goto IL_0cad;
                    }

                    if (num10 == 12) {
                        while (!Monitor.TryEnter(m_bufferLock, SimulationManager.SYNCHRONIZE_TIMEOUT)) {
                        }

                        uint num15 = 0u;
                        try {
                            if (m_pathUnits.CreateItem(out num15, ref m_pathRandomizer)) {
                                m_pathUnits.m_buffer[num15] = m_pathUnits.m_buffer[num9];
                                m_pathUnits.m_buffer[num15].m_referenceCount = 1;
                                m_pathUnits.m_buffer[num15].m_pathFindFlags = 4;
                                m_pathUnits.m_buffer[num9].m_nextPathUnit = num15;
                                m_pathUnits.m_buffer[num9].m_positionCount = (byte) num10;
                                num11 += num10;
                                Singleton<PathManager>.instance.m_pathUnitCount = (int) (m_pathUnits.ItemCount() - 1);
                                goto end_IL_0b9d;
                            }

                            m_pathUnits.m_buffer[unit].m_pathFindFlags |= 8;
                            return;
                            end_IL_0b9d: ;
                        } finally {
                            Monitor.Exit(m_bufferLock);
                        }

                        num9 = num15;
                        num10 = 0;
                    }

                    uint laneID = PathManager.GetLaneID(position);
                    position = m_laneTarget[laneID];
                    continue;
                    IL_0cad:
                    m_pathUnits.m_buffer[num9].m_positionCount = (byte) num10;
                    num11 += num10;
                    if (num11 != 0) {
                        num9 = m_pathUnits.m_buffer[unit].m_nextPathUnit;
                        num10 = m_pathUnits.m_buffer[unit].m_positionCount;
                        int num16 = 0;
                        while (num9 != 0) {
                            m_pathUnits.m_buffer[num9].m_length = num8 * (float) (num11 - num10) / (float) num11;
                            num10 += m_pathUnits.m_buffer[num9].m_positionCount;
                            num9 = m_pathUnits.m_buffer[num9].m_nextPathUnit;
                            if (++num16 >= 262144) {
                                CODebugBase<LogChannel>.Error(LogChannel.Core, "Invalid list detected!\n" + Environment.StackTrace);
                                break;
                            }
                        }
                    }

                    m_pathUnits.m_buffer[unit].m_pathFindFlags |= 4;
                    return;
                }

                m_pathUnits.m_buffer[unit].m_pathFindFlags |= 8;
            }
        }

        private void ProcessItemMain(BufferItem item, ushort nextNodeId, ref NetNode nextNode, byte connectOffset, bool isMiddle) {
            NetManager instance = Singleton<NetManager>.instance;
            bool flag = false;
            bool flag2 = false;
            bool flag3 = false;
            bool flag4 = false;
            int num = 0;
            NetInfo info = instance.m_segments.m_buffer[item.m_position.m_segment].Info;
            if (item.m_position.m_lane < info.m_lanes.Length) {
                NetInfo.Lane lane = info.m_lanes[item.m_position.m_lane];
                flag = (lane.m_laneType == NetInfo.LaneType.Pedestrian);
                flag2 = (lane.m_laneType == NetInfo.LaneType.Vehicle && (lane.m_vehicleType & m_vehicleTypes) == VehicleInfo.VehicleType.Bicycle);
                flag3 = lane.m_centerPlatform;
                flag4 = lane.m_elevated;
                num = (((lane.m_finalDirection & NetInfo.Direction.Forward) == NetInfo.Direction.None) ? (lane.m_similarLaneCount - lane.m_similarLaneIndex - 1) : lane.m_similarLaneIndex);
            }

            if (isMiddle) {
                for (int i = 0; i < 8; i++) {
                    ushort segment = nextNode.GetSegment(i);
                    if (segment != 0) {
                        ProcessItemCosts(item, nextNodeId, segment, ref instance.m_segments.m_buffer[segment], ref num, connectOffset, !flag, flag);
                    }
                }
            } else if (flag) {
                if (!flag4) {
                    ushort segment2 = item.m_position.m_segment;
                    int lane2 = item.m_position.m_lane;
                    if (nextNode.Info.m_class.m_service != ItemClass.Service.Beautification) {
                        bool flag5 = (nextNode.m_flags & (NetNode.Flags.End | NetNode.Flags.Bend | NetNode.Flags.Junction)) != NetNode.Flags.None;
                        bool flag6 = flag3 && (nextNode.m_flags & (NetNode.Flags.End | NetNode.Flags.Junction)) == NetNode.Flags.None;
                        ushort num2 = segment2;
                        ushort num3 = segment2;
                        int nextLaneIndex = 0;
                        int nextLaneIndex2 = 0;
                        uint num4 = 0u;
                        uint num5 = 0u;
                        instance.m_segments.m_buffer[segment2].GetLeftAndRightLanes(nextNodeId, NetInfo.LaneType.Pedestrian, VehicleInfo.VehicleType.None, lane2, flag6, out nextLaneIndex, out nextLaneIndex2, out num4, out num5);
                        if (num4 == 0 || num5 == 0) {
                            ushort num6 = 0;
                            ushort num7 = 0;
                            instance.m_segments.m_buffer[segment2].GetLeftAndRightSegments(nextNodeId, out num6, out num7);
                            int num8 = 0;
                            while (num6 != 0 && num6 != segment2 && num4 == 0) {
                                int num9 = 0;
                                int num10 = 0;
                                uint num11 = 0u;
                                uint num12 = 0u;
                                instance.m_segments.m_buffer[num6].GetLeftAndRightLanes(nextNodeId, NetInfo.LaneType.Pedestrian, VehicleInfo.VehicleType.None, -1, flag6, out num9, out num10, out num11, out num12);
                                if (num12 != 0) {
                                    num2 = num6;
                                    nextLaneIndex = num10;
                                    num4 = num12;
                                } else {
                                    num6 = instance.m_segments.m_buffer[num6].GetLeftSegment(nextNodeId);
                                }

                                if (++num8 == 8) {
                                    break;
                                }
                            }

                            num8 = 0;
                            while (num7 != 0 && num7 != segment2 && num5 == 0) {
                                int num13 = 0;
                                int num14 = 0;
                                uint num15 = 0u;
                                uint num16 = 0u;
                                instance.m_segments.m_buffer[num7].GetLeftAndRightLanes(nextNodeId, NetInfo.LaneType.Pedestrian, VehicleInfo.VehicleType.None, -1, flag6, out num13, out num14, out num15, out num16);
                                if (num15 != 0) {
                                    num3 = num7;
                                    nextLaneIndex2 = num13;
                                    num5 = num15;
                                } else {
                                    num7 = instance.m_segments.m_buffer[num7].GetRightSegment(nextNodeId);
                                }

                                if (++num8 == 8) {
                                    break;
                                }
                            }
                        }

                        if (num4 != 0 && (num2 != segment2 | flag5 | flag6)) {
                            ProcessItemPedBicycle(item, nextNodeId, num2, ref instance.m_segments.m_buffer[num2], connectOffset, connectOffset, nextLaneIndex, num4);
                        }

                        if (num5 != 0 && num5 != num4 && (num3 != segment2 | flag5 | flag6)) {
                            ProcessItemPedBicycle(item, nextNodeId, num3, ref instance.m_segments.m_buffer[num3], connectOffset, connectOffset, nextLaneIndex2, num5);
                        }

                        int nextLaneIndex3 = 0;
                        uint nextLaneId = 0u;
                        if ((m_vehicleTypes & VehicleInfo.VehicleType.Bicycle) != 0 && instance.m_segments.m_buffer[segment2].GetClosestLane((int) item.m_position.m_lane, NetInfo.LaneType.Vehicle, VehicleInfo.VehicleType.Bicycle, out nextLaneIndex3, out nextLaneId)) {
                            ProcessItemPedBicycle(item, nextNodeId, segment2, ref instance.m_segments.m_buffer[segment2], connectOffset, connectOffset, nextLaneIndex3, nextLaneId);
                        }
                    } else {
                        for (int j = 0; j < 8; j++) {
                            ushort segment3 = nextNode.GetSegment(j);
                            if (segment3 != 0 && segment3 != segment2) {
                                ProcessItemCosts(item, nextNodeId, segment3, ref instance.m_segments.m_buffer[segment3], ref num, connectOffset, false, true);
                            }
                        }
                    }

                    NetInfo.LaneType laneType = m_laneTypes & ~NetInfo.LaneType.Pedestrian;
                    VehicleInfo.VehicleType vehicleType = m_vehicleTypes & ~VehicleInfo.VehicleType.Bicycle;
                    if ((item.m_lanesUsed & (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle)) != 0) {
                        laneType &= ~(NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle);
                    }

                    int num17 = 0;
                    uint nextLaneId2 = 0u;
                    if (laneType != 0 && vehicleType != 0 && instance.m_segments.m_buffer[segment2].GetClosestLane(lane2, laneType, vehicleType, out num17, out nextLaneId2)) {
                        NetInfo.Lane lane3 = info.m_lanes[num17];
                        byte connectOffset2 = (byte) (((instance.m_segments.m_buffer[segment2].m_flags & NetSegment.Flags.Invert) != NetSegment.Flags.None == ((lane3.m_finalDirection & NetInfo.Direction.Backward) != NetInfo.Direction.None)) ? 1 : 254);
                        BufferItem item2 = item;
                        if (m_randomParking) {
                            item2.m_comparisonValue += (float) m_pathRandomizer.Int32(300u) / m_maxLength;
                        }

                        ProcessItemPedBicycle(item2, nextNodeId, segment2, ref instance.m_segments.m_buffer[segment2], connectOffset2, (byte) 128, num17, nextLaneId2);
                    }
                }
            } else {
                bool flag7 = (m_laneTypes & NetInfo.LaneType.Pedestrian) != NetInfo.LaneType.None;
                bool enablePedestrian = false;
                byte b = 0;
                if (flag7) {
                    if (flag2) {
                        b = connectOffset;
                        enablePedestrian = (nextNode.Info.m_class.m_service == ItemClass.Service.Beautification);
                    } else if (m_vehicleLane != 0) {
                        if (m_vehicleLane != item.m_laneID) {
                            flag7 = false;
                        } else {
                            b = m_vehicleOffset;
                        }
                    } else {
                        b = (byte) ((!m_stablePath) ? ((byte) m_pathRandomizer.UInt32(1u, 254u)) : 128);
                    }
                }

                ushort num18 = 0;
                if ((m_vehicleTypes & (VehicleInfo.VehicleType.Ferry | VehicleInfo.VehicleType.Monorail)) != 0) {
                    bool flag8 = (nextNode.m_flags & (NetNode.Flags.End | NetNode.Flags.Bend | NetNode.Flags.Junction)) != NetNode.Flags.None;
                    for (int k = 0; k < 8; k++) {
                        num18 = nextNode.GetSegment(k);
                        if (num18 != 0 && num18 != item.m_position.m_segment) {
                            ProcessItemCosts(item, nextNodeId, num18, ref instance.m_segments.m_buffer[num18], ref num, connectOffset, true, enablePedestrian);
                        }
                    }

                    if (flag8 && (m_vehicleTypes & VehicleInfo.VehicleType.Monorail) == VehicleInfo.VehicleType.None) {
                        num18 = item.m_position.m_segment;
                        ProcessItemCosts(item, nextNodeId, num18, ref instance.m_segments.m_buffer[num18], ref num, connectOffset, true, false);
                    }
                } else {
                    bool flag9 = (nextNode.m_flags & (NetNode.Flags.End | NetNode.Flags.OneWayOut)) != NetNode.Flags.None;
                    num18 = instance.m_segments.m_buffer[item.m_position.m_segment].GetRightSegment(nextNodeId);
                    for (int l = 0; l < 8; l++) {
                        if (num18 == 0) {
                            break;
                        }

                        if (num18 == item.m_position.m_segment) {
                            break;
                        }

                        if (ProcessItemCosts(item, nextNodeId, num18, ref instance.m_segments.m_buffer[num18], ref num, connectOffset, true, enablePedestrian)) {
                            flag9 = true;
                        }

                        num18 = instance.m_segments.m_buffer[num18].GetRightSegment(nextNodeId);
                    }

                    if (flag9 && (m_vehicleTypes & VehicleInfo.VehicleType.Tram) == VehicleInfo.VehicleType.None) {
                        num18 = item.m_position.m_segment;
                        ProcessItemCosts(item, nextNodeId, num18, ref instance.m_segments.m_buffer[num18], ref num, connectOffset, true, false);
                    }
                }

                if (flag7) {
                    num18 = item.m_position.m_segment;
                    int nextLaneIndex4 = 0;
                    uint nextLaneId3 = 0u;
                    if (instance.m_segments.m_buffer[num18].GetClosestLane((int) item.m_position.m_lane, NetInfo.LaneType.Pedestrian, m_vehicleTypes, out nextLaneIndex4, out nextLaneId3)) {
                        ProcessItemPedBicycle(item, nextNodeId, num18, ref instance.m_segments.m_buffer[num18], b, b, nextLaneIndex4, nextLaneId3);
                    }
                }
            }

            if (nextNode.m_lane != 0) {
                bool targetDisabled = (nextNode.m_flags & (NetNode.Flags.Disabled | NetNode.Flags.DisableOnlyMiddle)) == NetNode.Flags.Disabled;
                ushort segment4 = instance.m_lanes.m_buffer[nextNode.m_lane].m_segment;
                if (segment4 != 0 && segment4 != item.m_position.m_segment) {
                    ProcessItemPublicTransport(item, nextNodeId, targetDisabled, segment4, ref instance.m_segments.m_buffer[segment4], nextNode.m_lane, nextNode.m_laneOffset, connectOffset);
                }
            }
        }

        private void ProcessItemPublicTransport(BufferItem item, ushort nextNodeId, bool targetDisabled, ushort nextSegmentId, ref NetSegment nextSegment, uint nextLaneId, byte offset, byte connectOffset) {
            if ((nextSegment.m_flags & m_disableMask) == NetSegment.Flags.None) {
                NetManager instance = Singleton<NetManager>.instance;
                if (targetDisabled && ((instance.m_nodes.m_buffer[nextSegment.m_startNode].m_flags | instance.m_nodes.m_buffer[nextSegment.m_endNode].m_flags) & NetNode.Flags.Disabled) == NetNode.Flags.None) {
                    return;
                }

                NetInfo info = nextSegment.Info;
                NetInfo info2 = instance.m_segments.m_buffer[item.m_position.m_segment].Info;
                int num = info.m_lanes.Length;
                uint num2 = nextSegment.m_lanes;
                float num3 = 1f;
                float num4 = 1f;
                NetInfo.LaneType laneType = NetInfo.LaneType.None;
                if (item.m_position.m_lane < info2.m_lanes.Length) {
                    NetInfo.Lane lane = info2.m_lanes[item.m_position.m_lane];
                    num3 = lane.m_speedLimit;
                    laneType = lane.m_laneType;
                    if ((laneType & (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle)) != 0) {
                        laneType = (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle);
                    }

                    num4 = CalculateLaneSpeed(connectOffset, item.m_position.m_offset, ref instance.m_segments.m_buffer[item.m_position.m_segment], lane);
                }

                float num5 = (laneType != NetInfo.LaneType.PublicTransport) ? instance.m_segments.m_buffer[item.m_position.m_segment].m_averageLength : instance.m_lanes.m_buffer[item.m_laneID].m_length;
                float num6 = (float) Mathf.Abs(connectOffset - item.m_position.m_offset) * 0.003921569f * num5;
                float num7 = item.m_methodDistance + num6;
                float num8 = item.m_comparisonValue + num6 / (num4 * m_maxLength);
                float num9 = item.m_duration + num6 / num3;
                Vector3 b = instance.m_lanes.m_buffer[item.m_laneID].CalculatePosition((float) (int) connectOffset * 0.003921569f);
                if (!m_ignoreCost) {
                    int ticketCost = instance.m_lanes.m_buffer[item.m_laneID].m_ticketCost;
                    if (ticketCost != 0) {
                        num8 += (float) (ticketCost * m_pathRandomizer.Int32(2000u)) * 3.92156863E-07f;
                    }
                }

                int num10 = 0;
                while (true) {
                    if (num10 >= num) {
                        return;
                    }

                    if (num2 == 0) {
                        return;
                    }

                    if (nextLaneId == num2) {
                        break;
                    }

                    num2 = instance.m_lanes.m_buffer[num2].m_nextLane;
                    num10++;
                }

                NetInfo.Lane lane2 = info.m_lanes[num10];
                if (lane2.CheckType(m_laneTypes, m_vehicleTypes)) {
                    float num11 = Vector3.Distance(instance.m_lanes.m_buffer[nextLaneId].CalculatePosition((float) (int) offset * 0.003921569f), b);
                    BufferItem bufferItem = default(BufferItem);
                    bufferItem.m_position.m_segment = nextSegmentId;
                    bufferItem.m_position.m_lane = (byte) num10;
                    bufferItem.m_position.m_offset = offset;
                    if ((lane2.m_laneType & laneType) == NetInfo.LaneType.None) {
                        bufferItem.m_methodDistance = 0f;
                    } else {
                        bufferItem.m_methodDistance = num7 + num11;
                    }

                    if (lane2.m_laneType == NetInfo.LaneType.Pedestrian && !(bufferItem.m_methodDistance < 1000f) && !m_stablePath) {
                        return;
                    }

                    bufferItem.m_comparisonValue = num8 + num11 / ((num3 + lane2.m_speedLimit) * 0.5f * m_maxLength);
                    bufferItem.m_duration = num9 + num11 / ((num3 + lane2.m_speedLimit) * 0.5f);
                    if ((nextSegment.m_flags & NetSegment.Flags.Invert) != 0) {
                        bufferItem.m_direction = NetInfo.InvertDirection(lane2.m_finalDirection);
                    } else {
                        bufferItem.m_direction = lane2.m_finalDirection;
                    }

                    if (nextLaneId == m_startLaneA) {
                        if ((bufferItem.m_direction & NetInfo.Direction.Forward) == NetInfo.Direction.None || bufferItem.m_position.m_offset < m_startOffsetA) {
                            if ((bufferItem.m_direction & NetInfo.Direction.Backward) == NetInfo.Direction.None) {
                                return;
                            }

                            if (bufferItem.m_position.m_offset > m_startOffsetA) {
                                return;
                            }
                        }

                        float num12 = CalculateLaneSpeed(m_startOffsetA, bufferItem.m_position.m_offset, ref nextSegment, lane2);
                        float num13 = (float) Mathf.Abs(bufferItem.m_position.m_offset - m_startOffsetA) * 0.003921569f;
                        bufferItem.m_comparisonValue += num13 * nextSegment.m_averageLength / (num12 * m_maxLength);
                        bufferItem.m_duration += num13 * nextSegment.m_averageLength / num12;
                    }

                    if (nextLaneId == m_startLaneB) {
                        if ((bufferItem.m_direction & NetInfo.Direction.Forward) == NetInfo.Direction.None || bufferItem.m_position.m_offset < m_startOffsetB) {
                            if ((bufferItem.m_direction & NetInfo.Direction.Backward) == NetInfo.Direction.None) {
                                return;
                            }

                            if (bufferItem.m_position.m_offset > m_startOffsetB) {
                                return;
                            }
                        }

                        float num14 = CalculateLaneSpeed(m_startOffsetB, bufferItem.m_position.m_offset, ref nextSegment, lane2);
                        float num15 = (float) Mathf.Abs(bufferItem.m_position.m_offset - m_startOffsetB) * 0.003921569f;
                        bufferItem.m_comparisonValue += num15 * nextSegment.m_averageLength / (num14 * m_maxLength);
                        bufferItem.m_duration += num15 * nextSegment.m_averageLength / num14;
                    }

                    bufferItem.m_laneID = nextLaneId;
                    bufferItem.m_lanesUsed = (item.m_lanesUsed | lane2.m_laneType);
                    AddBufferItem(bufferItem, item.m_position);
                }
            }
        }

        private bool ProcessItemCosts(BufferItem item, ushort nextNodeId, ushort nextSegmentId, ref NetSegment nextSegment, ref int laneIndexFromInner, byte connectOffset, bool enableVehicle, bool enablePedestrian) {
            bool result = false;
            if ((nextSegment.m_flags & m_disableMask) != 0) {
                return result;
            }

            NetManager instance = Singleton<NetManager>.instance;
            NetInfo info = nextSegment.Info;
            NetInfo info2 = instance.m_segments.m_buffer[item.m_position.m_segment].Info;
            int num = info.m_lanes.Length;
            uint num2 = nextSegment.m_lanes;
            NetInfo.Direction direction = (NetInfo.Direction) ((nextNodeId != nextSegment.m_startNode) ? 1 : 2);
            NetInfo.Direction direction2 = ((nextSegment.m_flags & NetSegment.Flags.Invert) == NetSegment.Flags.None) ? direction : NetInfo.InvertDirection(direction);
            float num3 = 1f;
            float num4 = 1f;
            NetInfo.LaneType laneType = NetInfo.LaneType.None;
            VehicleInfo.VehicleType vehicleType = VehicleInfo.VehicleType.None;
            if (item.m_position.m_lane < info2.m_lanes.Length) {
                NetInfo.Lane lane = info2.m_lanes[item.m_position.m_lane];
                laneType = lane.m_laneType;
                vehicleType = lane.m_vehicleType;
                num3 = lane.m_speedLimit;
                num4 = CalculateLaneSpeed(connectOffset, item.m_position.m_offset, ref instance.m_segments.m_buffer[item.m_position.m_segment], lane);
            }

            bool flag = false;
            if (laneType == NetInfo.LaneType.Vehicle && (vehicleType & VehicleInfo.VehicleType.Car) == VehicleInfo.VehicleType.None) {
                float num5 = 0.01f - Mathf.Min(info.m_maxTurnAngleCos, info2.m_maxTurnAngleCos);
                if (num5 < 1f) {
                    Vector3 vector = (nextNodeId != instance.m_segments.m_buffer[item.m_position.m_segment].m_startNode) ? instance.m_segments.m_buffer[item.m_position.m_segment].m_endDirection : instance.m_segments.m_buffer[item.m_position.m_segment].m_startDirection;
                    Vector3 vector2 = ((direction & NetInfo.Direction.Forward) == NetInfo.Direction.None) ? nextSegment.m_startDirection : nextSegment.m_endDirection;
                    if (vector.x * vector2.x + vector.z * vector2.z >= num5) {
                        flag = true;
                    }
                }
            }

            float num6 = (laneType != NetInfo.LaneType.PublicTransport) ? instance.m_segments.m_buffer[item.m_position.m_segment].m_averageLength : instance.m_lanes.m_buffer[item.m_laneID].m_length;
            float num7 = (float) Mathf.Abs(connectOffset - item.m_position.m_offset) * 0.003921569f * num6;
            float num8 = item.m_methodDistance + num7;
            float num9 = item.m_duration + num7 / num3;
            if (!m_stablePath) {
                num7 *= (float) (new Randomizer(m_pathFindIndex << 16 | item.m_position.m_segment).Int32(900, 1000 + instance.m_segments.m_buffer[item.m_position.m_segment].m_trafficDensity * 10) + m_pathRandomizer.Int32(20u)) * 0.001f;
            }

            if ((laneType & (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle)) != 0 && (vehicleType & m_vehicleTypes) == VehicleInfo.VehicleType.Car && (instance.m_segments.m_buffer[item.m_position.m_segment].m_flags & m_carBanMask) != 0) {
                num7 *= 7.5f;
            }

            if (m_transportVehicle && laneType == NetInfo.LaneType.TransportVehicle) {
                num7 *= 0.95f;
            }

            float num10 = item.m_comparisonValue + num7 / (num4 * m_maxLength);
            if (!m_ignoreCost) {
                int ticketCost = instance.m_lanes.m_buffer[item.m_laneID].m_ticketCost;
                if (ticketCost != 0) {
                    num10 += (float) (ticketCost * m_pathRandomizer.Int32(2000u)) * 3.92156863E-07f;
                }
            }

            if ((laneType & (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle)) != 0) {
                laneType = (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle);
            }

            Vector3 b = instance.m_lanes.m_buffer[item.m_laneID].CalculatePosition((float) (int) connectOffset * 0.003921569f);
            int num11 = laneIndexFromInner;
            bool flag2 = (instance.m_nodes.m_buffer[nextNodeId].m_flags & NetNode.Flags.Transition) != NetNode.Flags.None;
            NetInfo.LaneType laneType2 = m_laneTypes;
            VehicleInfo.VehicleType vehicleType2 = m_vehicleTypes;
            if (!enableVehicle) {
                vehicleType2 &= VehicleInfo.VehicleType.Bicycle;
                if (vehicleType2 == VehicleInfo.VehicleType.None) {
                    laneType2 &= ~(NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle);
                }
            }

            if (!enablePedestrian) {
                laneType2 &= ~NetInfo.LaneType.Pedestrian;
            }

            for (int i = 0; i < num && num2 != 0; i++) {
                NetInfo.Lane lane2 = info.m_lanes[i];
                BufferItem bufferItem = default(BufferItem);
                float num12;
                if ((lane2.m_finalDirection & direction2) != 0) {
                    if (lane2.CheckType(laneType2, vehicleType2) && (nextSegmentId != item.m_position.m_segment || i != item.m_position.m_lane) && (lane2.m_finalDirection & direction2) != 0) {
                        if (flag && lane2.m_laneType == NetInfo.LaneType.Vehicle && (lane2.m_vehicleType & VehicleInfo.VehicleType.Car) == VehicleInfo.VehicleType.None) {
                            continue;
                        }

                        num12 = Vector3.Distance(((direction & NetInfo.Direction.Forward) == NetInfo.Direction.None) ? instance.m_lanes.m_buffer[num2].m_bezier.a : instance.m_lanes.m_buffer[num2].m_bezier.d, b);
                        if (flag2) {
                            num12 *= 2f;
                        }

                        float num13 = num12 / ((num3 + lane2.m_speedLimit) * 0.5f * m_maxLength);
                        bufferItem.m_position.m_segment = nextSegmentId;
                        bufferItem.m_position.m_lane = (byte) i;
                        bufferItem.m_position.m_offset = (byte) (((direction & NetInfo.Direction.Forward) != 0) ? 255 : 0);
                        if ((lane2.m_laneType & laneType) == NetInfo.LaneType.None) {
                            bufferItem.m_methodDistance = 0f;
                        } else {
                            bufferItem.m_methodDistance = num8 + num12;
                        }

                        if (lane2.m_laneType == NetInfo.LaneType.Pedestrian && !(bufferItem.m_methodDistance < 1000f) && !m_stablePath) {
                            goto IL_06d3;
                        }

                        bufferItem.m_comparisonValue = num10 + num13;
                        bufferItem.m_duration = num9 + num12 / ((num3 + lane2.m_speedLimit) * 0.5f);
                        bufferItem.m_direction = direction;
                        if (num2 == m_startLaneA) {
                            if ((bufferItem.m_direction & NetInfo.Direction.Forward) != 0 && bufferItem.m_position.m_offset >= m_startOffsetA) {
                                goto IL_0658;
                            }

                            if ((bufferItem.m_direction & NetInfo.Direction.Backward) != 0 && bufferItem.m_position.m_offset <= m_startOffsetA) {
                                goto IL_0658;
                            }

                            num2 = instance.m_lanes.m_buffer[num2].m_nextLane;
                            continue;
                        }

                        goto IL_082f;
                    }
                } else if ((lane2.m_laneType & laneType) != 0 && (lane2.m_vehicleType & vehicleType) != 0) {
                    num11++;
                }

                goto IL_06d3;
                IL_0658:
                float num14 = CalculateLaneSpeed(m_startOffsetA, bufferItem.m_position.m_offset, ref nextSegment, lane2);
                float num15 = (float) Mathf.Abs(bufferItem.m_position.m_offset - m_startOffsetA) * 0.003921569f;
                bufferItem.m_comparisonValue += num15 * nextSegment.m_averageLength / (num14 * m_maxLength);
                bufferItem.m_duration += num15 * nextSegment.m_averageLength / num14;
                goto IL_082f;
                IL_06d3:
                num2 = instance.m_lanes.m_buffer[num2].m_nextLane;
                continue;
                IL_06f1:
                if (!m_ignoreBlocked && (nextSegment.m_flags & NetSegment.Flags.Blocked) != 0 && (lane2.m_laneType & (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle)) != 0) {
                    bufferItem.m_comparisonValue += 0.1f;
                    result = true;
                }

                bufferItem.m_lanesUsed = (item.m_lanesUsed | lane2.m_laneType);
                bufferItem.m_laneID = num2;
                if ((lane2.m_laneType & laneType) != 0 && (lane2.m_vehicleType & m_vehicleTypes) != 0) {
                    int firstTarget = instance.m_lanes.m_buffer[num2].m_firstTarget;
                    int lastTarget = instance.m_lanes.m_buffer[num2].m_lastTarget;
                    if (laneIndexFromInner < firstTarget || laneIndexFromInner >= lastTarget) {
                        bufferItem.m_comparisonValue += Mathf.Max(1f, num12 * 3f - 3f) / ((num3 + lane2.m_speedLimit) * 0.5f * m_maxLength);
                    }

                    if (!m_transportVehicle && lane2.m_laneType == NetInfo.LaneType.TransportVehicle) {
                        bufferItem.m_comparisonValue += 20f / ((num3 + lane2.m_speedLimit) * 0.5f * m_maxLength);
                    }
                }

                AddBufferItem(bufferItem, item.m_position);
                goto IL_06d3;
                IL_082f:
                if (num2 != m_startLaneB) {
                    goto IL_06f1;
                }

                if ((bufferItem.m_direction & NetInfo.Direction.Forward) != 0 && bufferItem.m_position.m_offset >= m_startOffsetB) {
                    goto IL_0895;
                }

                if ((bufferItem.m_direction & NetInfo.Direction.Backward) != 0 && bufferItem.m_position.m_offset <= m_startOffsetB) {
                    goto IL_0895;
                }

                num2 = instance.m_lanes.m_buffer[num2].m_nextLane;
                continue;
                IL_0895:
                float num16 = CalculateLaneSpeed(m_startOffsetB, bufferItem.m_position.m_offset, ref nextSegment, lane2);
                float num17 = (float) Mathf.Abs(bufferItem.m_position.m_offset - m_startOffsetB) * 0.003921569f;
                bufferItem.m_comparisonValue += num17 * nextSegment.m_averageLength / (num16 * m_maxLength);
                bufferItem.m_duration += num17 * nextSegment.m_averageLength / num16;
                goto IL_06f1;
            }

            laneIndexFromInner = num11;
            return result;
        }

        private void ProcessItemPedBicycle(BufferItem item, ushort nextNodeId, ushort nextSegmentId, ref NetSegment nextSegment, byte connectOffset, byte laneSwitchOffset, int nextLaneIndex, uint nextLaneId) {
            NetInfo.Lane lane2;
            BufferItem bufferItem;
            if ((nextSegment.m_flags & m_disableMask) == NetSegment.Flags.None) {
                NetManager instance = Singleton<NetManager>.instance;
                NetInfo info = nextSegment.Info;
                NetInfo info2 = instance.m_segments.m_buffer[item.m_position.m_segment].Info;
                int num = info.m_lanes.Length;
                float num2;
                byte offset;
                if (nextSegmentId == item.m_position.m_segment) {
                    Vector3 b = instance.m_lanes.m_buffer[item.m_laneID].CalculatePosition((float) (int) laneSwitchOffset * 0.003921569f);
                    num2 = Vector3.Distance(instance.m_lanes.m_buffer[nextLaneId].CalculatePosition((float) (int) connectOffset * 0.003921569f), b);
                    offset = connectOffset;
                } else {
                    byte num3 = (byte) ((nextNodeId != nextSegment.m_startNode) ? 1 : 2);
                    Vector3 b2 = instance.m_lanes.m_buffer[item.m_laneID].CalculatePosition((float) (int) laneSwitchOffset * 0.003921569f);
                    num2 = Vector3.Distance(((num3 & 1) == 0) ? instance.m_lanes.m_buffer[nextLaneId].m_bezier.a : instance.m_lanes.m_buffer[nextLaneId].m_bezier.d, b2);
                    offset = (byte) (((num3 & 1) != 0) ? 255 : 0);
                }

                float num4 = 1f;
                float num5 = 1f;
                NetInfo.LaneType laneType = NetInfo.LaneType.None;
                if (item.m_position.m_lane < info2.m_lanes.Length) {
                    NetInfo.Lane lane = info2.m_lanes[item.m_position.m_lane];
                    num4 = lane.m_speedLimit;
                    laneType = lane.m_laneType;
                    if ((laneType & (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle)) != 0) {
                        laneType = (NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle);
                    }

                    num5 = CalculateLaneSpeed(laneSwitchOffset, item.m_position.m_offset, ref instance.m_segments.m_buffer[item.m_position.m_segment], lane);
                }

                float num6 = (laneType != NetInfo.LaneType.PublicTransport) ? instance.m_segments.m_buffer[item.m_position.m_segment].m_averageLength : instance.m_lanes.m_buffer[item.m_laneID].m_length;
                float num7 = (float) Mathf.Abs(laneSwitchOffset - item.m_position.m_offset) * 0.003921569f * num6;
                float num8 = item.m_methodDistance + num7;
                float num9 = item.m_comparisonValue + num7 / (num5 * m_maxLength);
                float num10 = item.m_duration + num7 / num4;
                if (!m_ignoreCost) {
                    int ticketCost = instance.m_lanes.m_buffer[item.m_laneID].m_ticketCost;
                    if (ticketCost != 0) {
                        num9 += (float) (ticketCost * m_pathRandomizer.Int32(2000u)) * 3.92156863E-07f;
                    }
                }

                if (nextLaneIndex >= num) {
                    return;
                }

                lane2 = info.m_lanes[nextLaneIndex];
                bufferItem = default(BufferItem);
                bufferItem.m_position.m_segment = nextSegmentId;
                bufferItem.m_position.m_lane = (byte) nextLaneIndex;
                bufferItem.m_position.m_offset = offset;
                if ((lane2.m_laneType & laneType) == NetInfo.LaneType.None) {
                    bufferItem.m_methodDistance = 0f;
                } else {
                    if (item.m_methodDistance == 0f) {
                        num9 += 100f / (0.25f * m_maxLength);
                    }

                    bufferItem.m_methodDistance = num8 + num2;
                }

                if (lane2.m_laneType == NetInfo.LaneType.Pedestrian && !(bufferItem.m_methodDistance < 1000f) && !m_stablePath) {
                    return;
                }

                bufferItem.m_comparisonValue = num9 + num2 / ((num4 + lane2.m_speedLimit) * 0.25f * m_maxLength);
                bufferItem.m_duration = num10 + num2 / ((num4 + lane2.m_speedLimit) * 0.5f);
                if ((nextSegment.m_flags & NetSegment.Flags.Invert) != 0) {
                    bufferItem.m_direction = NetInfo.InvertDirection(lane2.m_finalDirection);
                } else {
                    bufferItem.m_direction = lane2.m_finalDirection;
                }

                if (nextLaneId == m_startLaneA) {
                    if ((bufferItem.m_direction & NetInfo.Direction.Forward) != 0 && bufferItem.m_position.m_offset >= m_startOffsetA) {
                        goto IL_040a;
                    }

                    if ((bufferItem.m_direction & NetInfo.Direction.Backward) != 0 && bufferItem.m_position.m_offset <= m_startOffsetA) {
                        goto IL_040a;
                    }

                    return;
                }

                goto IL_0480;
            }

            return;
            IL_0480:
            if (nextLaneId == m_startLaneB) {
                if ((bufferItem.m_direction & NetInfo.Direction.Forward) != 0 && bufferItem.m_position.m_offset >= m_startOffsetB) {
                    goto IL_04cd;
                }

                if ((bufferItem.m_direction & NetInfo.Direction.Backward) != 0 && bufferItem.m_position.m_offset <= m_startOffsetB) {
                    goto IL_04cd;
                }

                return;
            }

            goto IL_0543;
            IL_0543:
            bufferItem.m_laneID = nextLaneId;
            bufferItem.m_lanesUsed = (item.m_lanesUsed | lane2.m_laneType);
            AddBufferItem(bufferItem, item.m_position);
            return;
            IL_040a:
            float num11 = CalculateLaneSpeed(m_startOffsetA, bufferItem.m_position.m_offset, ref nextSegment, lane2);
            float num12 = (float) Mathf.Abs(bufferItem.m_position.m_offset - m_startOffsetA) * 0.003921569f;
            bufferItem.m_comparisonValue += num12 * nextSegment.m_averageLength / (num11 * m_maxLength);
            bufferItem.m_duration += num12 * nextSegment.m_averageLength / num11;
            goto IL_0480;
            IL_04cd:
            float num13 = CalculateLaneSpeed(m_startOffsetB, bufferItem.m_position.m_offset, ref nextSegment, lane2);
            float num14 = (float) Mathf.Abs(bufferItem.m_position.m_offset - m_startOffsetB) * 0.003921569f;
            bufferItem.m_comparisonValue += num14 * nextSegment.m_averageLength / (num13 * m_maxLength);
            bufferItem.m_duration += num14 * nextSegment.m_averageLength / num13;
            goto IL_0543;
        }

        private void AddBufferItem(BufferItem item, PathUnit.Position target) {
            uint num = m_laneLocation[item.m_laneID];
            uint num2 = num >> 16;
            int num3 = (int) (num & 0xFFFF);
            int num6;
            if (num2 == m_pathFindIndex) {
                if (!(item.m_comparisonValue >= m_buffer[num3].m_comparisonValue)) {
                    int num4 = num3 >> 6;
                    int num5 = num3 & -64;
                    if (num4 < m_bufferMinPos) {
                        return;
                    }

                    if (num4 == m_bufferMinPos && num5 < m_bufferMin[num4]) {
                        return;
                    }

                    num6 = Mathf.Max(Mathf.RoundToInt(item.m_comparisonValue * 1024f), m_bufferMinPos);
                    if (num6 == num4) {
                        m_buffer[num3] = item;
                        m_laneTarget[item.m_laneID] = target;
                        return;
                    }

                    int num7 = num4 << 6 | m_bufferMax[num4]--;
                    BufferItem bufferItem = m_buffer[num7];
                    m_laneLocation[bufferItem.m_laneID] = num;
                    m_buffer[num3] = bufferItem;
                    goto IL_0111;
                }

                return;
            }

            num6 = Mathf.Max(Mathf.RoundToInt(item.m_comparisonValue * 1024f), m_bufferMinPos);
            goto IL_0111;
            IL_0111:
            if (num6 < 1024 && num6 >= 0) {
                while (m_bufferMax[num6] == 63) {
                    num6++;
                    if (num6 == 1024) {
                        return;
                    }
                }

                if (num6 > m_bufferMaxPos) {
                    m_bufferMaxPos = num6;
                }

                num3 = (num6 << 6 | ++m_bufferMax[num6]);
                m_buffer[num3] = item;
                m_laneLocation[item.m_laneID] = (uint) ((int) (m_pathFindIndex << 16) | num3);
                m_laneTarget[item.m_laneID] = target;
            }
        }

        private float CalculateLaneSpeed(byte startOffset, byte endOffset, ref NetSegment segment, NetInfo.Lane laneInfo) {
            NetInfo.Direction direction = ((segment.m_flags & NetSegment.Flags.Invert) == NetSegment.Flags.None) ? laneInfo.m_finalDirection : NetInfo.InvertDirection(laneInfo.m_finalDirection);
            if ((direction & NetInfo.Direction.Avoid) != 0) {
                if (endOffset > startOffset && direction == NetInfo.Direction.AvoidForward) {
                    return laneInfo.m_speedLimit * 0.1f;
                }

                if (endOffset < startOffset && direction == NetInfo.Direction.AvoidBackward) {
                    return laneInfo.m_speedLimit * 0.1f;
                }

                return laneInfo.m_speedLimit * 0.2f;
            }

            return laneInfo.m_speedLimit;
        }

        private void GetLaneDirection(PathUnit.Position pathPos, out NetInfo.Direction direction, out NetInfo.LaneType type) {
            NetManager instance = Singleton<NetManager>.instance;
            NetInfo info = instance.m_segments.m_buffer[pathPos.m_segment].Info;
            if (info.m_lanes.Length > pathPos.m_lane) {
                direction = info.m_lanes[pathPos.m_lane].m_finalDirection;
                type = info.m_lanes[pathPos.m_lane].m_laneType;
                if ((instance.m_segments.m_buffer[pathPos.m_segment].m_flags & NetSegment.Flags.Invert) != 0) {
                    direction = NetInfo.InvertDirection(direction);
                }
            } else {
                direction = NetInfo.Direction.None;
                type = NetInfo.LaneType.None;
            }
        }

        private void PathFindThread() {
            while (true) {
                if (Monitor.TryEnter(m_queueLock, SimulationManager.SYNCHRONIZE_TIMEOUT)) {
                    try {
                        while (m_queueFirst == 0 && !m_terminated) {
                            Monitor.Wait(m_queueLock);
                        }

                        if (m_terminated) {
                            return;
                        }

                        m_calculating = m_queueFirst;
                        m_queueFirst = m_pathUnits.m_buffer[m_calculating].m_nextPathUnit;
                        if (m_queueFirst == 0) {
                            m_queueLast = 0u;
                            m_queuedPathFindCount = 0;
                        } else {
                            m_queuedPathFindCount--;
                        }

                        m_pathUnits.m_buffer[m_calculating].m_nextPathUnit = 0u;
                        m_pathUnits.m_buffer[m_calculating].m_pathFindFlags = (byte) ((m_pathUnits.m_buffer[m_calculating].m_pathFindFlags & -2) | 2);
                    } finally {
                        Monitor.Exit(m_queueLock);
                    }

                    try {
                        m_pathfindProfiler.BeginStep();
                        try {
                            PathFindImplementation(m_calculating, ref m_pathUnits.m_buffer[m_calculating]);
                        } finally {
                            m_pathfindProfiler.EndStep();
                        }
                    } catch (Exception ex) {
                        UIView.ForwardException(ex);
                        CODebugBase<LogChannel>.Error(LogChannel.Core, "Path find error: " + ex.Message + "\n" + ex.StackTrace);
                        m_pathUnits.m_buffer[m_calculating].m_pathFindFlags |= 8;
                    }

                    while (!Monitor.TryEnter(m_queueLock, SimulationManager.SYNCHRONIZE_TIMEOUT)) {
                    }

                    try {
                        m_pathUnits.m_buffer[m_calculating].m_pathFindFlags = (byte) (m_pathUnits.m_buffer[m_calculating].m_pathFindFlags & -3);
                        Singleton<PathManager>.instance.ReleasePath(m_calculating);
                        m_calculating = 0u;
                        Monitor.Pulse(m_queueLock);
                    } finally {
                        Monitor.Exit(m_queueLock);
                    }
                }
            }
        }
    }
}