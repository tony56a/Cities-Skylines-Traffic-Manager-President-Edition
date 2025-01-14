﻿using ColossalFramework;
using CSUtil.Commons;
using System;
using System.Collections.Generic;
using System.Text;
using TrafficManager.Geometry;
using TrafficManager.Geometry.Impl;
using TrafficManager.State;
using TrafficManager.Traffic.Data;
using TrafficManager.Util;
using UnityEngine;

namespace TrafficManager.Manager.Impl {
	public class SpeedLimitManager : AbstractGeometryObservingManager, ICustomDataManager<List<Configuration.LaneSpeedLimit>>, ICustomDataManager<Dictionary<string, float>>, ISpeedLimitManager {
		public const NetInfo.LaneType LANE_TYPES = NetInfo.LaneType.Vehicle | NetInfo.LaneType.TransportVehicle;
		public const VehicleInfo.VehicleType VEHICLE_TYPES = VehicleInfo.VehicleType.Car | VehicleInfo.VehicleType.Tram | VehicleInfo.VehicleType.Metro | VehicleInfo.VehicleType.Train | VehicleInfo.VehicleType.Monorail;

		/// <summary>Ingame speed units, max possible speed</summary>
		public const float MAX_SPEED = 10f * 2f; // 1000 km/h

		private Dictionary<string, float[]> vanillaLaneSpeedLimitsByNetInfoName; // For each NetInfo (by name) and lane index: game default speed limit
		private Dictionary<string, List<string>> childNetInfoNamesByCustomizableNetInfoName; // For each NetInfo (by name): Parent NetInfo (name)
		private List<NetInfo> customizableNetInfos;

		internal Dictionary<string, float> CustomLaneSpeedLimitByNetInfoName; // For each NetInfo (by name) and lane index: custom speed limit
		internal Dictionary<string, NetInfo> NetInfoByName; // For each name: NetInfo

		public static readonly SpeedLimitManager Instance = new SpeedLimitManager();

		private SpeedLimitManager() {
			vanillaLaneSpeedLimitsByNetInfoName = new Dictionary<string, float[]>();
			CustomLaneSpeedLimitByNetInfoName = new Dictionary<string, float>();
			customizableNetInfos = new List<NetInfo>();
			childNetInfoNamesByCustomizableNetInfoName = new Dictionary<string, List<string>>();
			NetInfoByName = new Dictionary<string, NetInfo>();
		}

		protected override void InternalPrintDebugInfo() {
			base.InternalPrintDebugInfo();
			Log._Debug($"- Not implemented -");
			// TODO implement
		}

		/// <summary>
		/// Determines if custom speed limits may be assigned to the given segment.
		/// </summary>
		/// <param name="segmentId"></param>
		/// <param name="segment"></param>
		/// <returns></returns>
		public bool MayHaveCustomSpeedLimits(ushort segmentId, ref NetSegment segment) {
			if ((segment.m_flags & NetSegment.Flags.Created) == NetSegment.Flags.None)
				return false;
			ItemClass connectionClass = segment.Info.GetConnectionClass();
			return (connectionClass.m_service == ItemClass.Service.Road ||
				(connectionClass.m_service == ItemClass.Service.PublicTransport && (connectionClass.m_subService == ItemClass.SubService.PublicTransportTrain || connectionClass.m_subService == ItemClass.SubService.PublicTransportTram || connectionClass.m_subService == ItemClass.SubService.PublicTransportMetro || connectionClass.m_subService == ItemClass.SubService.PublicTransportMonorail)));
		}

		/// <summary>
		/// Determines if custom speed limits may be assigned to the given lane info
		/// </summary>
		/// <param name="laneInfo"></param>
		/// <returns></returns>
		public bool MayHaveCustomSpeedLimits(NetInfo.Lane laneInfo) {
			return (laneInfo.m_laneType & LANE_TYPES) != NetInfo.LaneType.None &&
					(laneInfo.m_vehicleType & VEHICLE_TYPES) != VehicleInfo.VehicleType.None;
		}

		/// <summary>
		/// Determines the currently set speed limit for the given segment and lane direction in terms of discrete speed limit levels.
		/// An in-game speed limit of 2.0 (e.g. on highway) is hereby translated into a discrete speed limit value of 100 (km/h).
		/// </summary>
		/// <param name="segmentId"></param>
		/// <param name="finalDir"></param>
		/// <returns>Mean speed limit, average for custom and default lane speeds</returns>
		public float GetCustomSpeedLimit(ushort segmentId, NetInfo.Direction finalDir) {
			// calculate the currently set mean speed limit
			if (segmentId == 0 || (Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].m_flags & NetSegment.Flags.Created) == NetSegment.Flags.None) {
				return 0.0f;
			}

			var segmentInfo = Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].Info;
			var curLaneId = Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].m_lanes;
			var laneIndex = 0;
			var meanSpeedLimit = 0f;
			uint validLanes = 0;
			while (laneIndex < segmentInfo.m_lanes.Length && curLaneId != 0u) {
				var laneInfo = segmentInfo.m_lanes[laneIndex];
				var d = laneInfo.m_finalDirection;
				if (d != finalDir) {
					goto nextIter;
				}
				if (!MayHaveCustomSpeedLimits(laneInfo)) {
					goto nextIter;
				}

				var setSpeedLimit = Flags.getLaneSpeedLimit(curLaneId);
				if (setSpeedLimit != null) {
					meanSpeedLimit += ToGameSpeedLimit(setSpeedLimit.Value); // custom speed limit
				} else {
					meanSpeedLimit += laneInfo.m_speedLimit; // game default
				}

				++validLanes;

			nextIter:
				curLaneId = Singleton<NetManager>.instance.m_lanes.m_buffer[curLaneId].m_nextLane;
				laneIndex++;
			}

			if (validLanes > 0) {
				meanSpeedLimit /= validLanes;
			}

			return meanSpeedLimit;
		}

		/// <summary>
		/// Determines the average default speed limit for a given NetInfo object in terms of discrete speed limit levels.
		/// An in-game speed limit of 2.0 (e.g. on highway) is hereby translated into a discrete speed limit value of 100 (km/h).
		/// </summary>
		/// <param name="segmentInfo"></param>
		/// <param name="finalDir"></param>
		/// <returns></returns>
		public float GetAverageDefaultCustomSpeedLimit(NetInfo segmentInfo, NetInfo.Direction? finalDir=null) {
			var meanSpeedLimit = 0f;
			uint validLanes = 0;
			foreach (var laneInfo in segmentInfo.m_lanes) {
				var d = laneInfo.m_finalDirection;
				if (finalDir != null && d != finalDir) {
					continue;
				}
				if (!MayHaveCustomSpeedLimits(laneInfo)) {
					continue;
				}

				meanSpeedLimit += laneInfo.m_speedLimit;
				++validLanes;
			}

			if (validLanes > 0) {
				meanSpeedLimit /= validLanes;
			}
			return meanSpeedLimit;
		}

        /// <summary>
		/// Determines the average custom speed limit for a given NetInfo object in terms of discrete speed limit levels.
		/// An in-game speed limit of 2.0 (e.g. on highway) is hereby translated into a discrete speed limit value of 100 (km/h).
		/// </summary>
		/// <param name="segmentInfo"></param>
		/// <param name="finalDir"></param>
		/// <returns></returns>
		public ushort GetAverageCustomSpeedLimit(ushort segmentId, ref NetSegment segment, NetInfo segmentInfo, NetInfo.Direction? finalDir = null) {
            // calculate the currently set mean speed limit
            float meanSpeedLimit = 0f;
            uint validLanes = 0;
            uint curLaneId = segment.m_lanes;
            for (byte laneIndex = 0; laneIndex < segmentInfo.m_lanes.Length; ++laneIndex) {
				NetInfo.Lane laneInfo = segmentInfo.m_lanes[laneIndex];
				NetInfo.Direction d = laneInfo.m_finalDirection;
				if (finalDir != null && d != finalDir)
					continue;
				if (!MayHaveCustomSpeedLimits(laneInfo))
					continue;

                meanSpeedLimit += GetLockFreeGameSpeedLimit(segmentId, laneIndex, curLaneId, laneInfo);
                curLaneId = Singleton<NetManager>.instance.m_lanes.m_buffer[curLaneId].m_nextLane;
                ++validLanes;
            }

            if (validLanes > 0)
                meanSpeedLimit /= (float)validLanes;
            return (ushort)Mathf.Round(meanSpeedLimit);
        }

        /// <summary>
        /// Determines the currently set speed limit for the given lane in terms of discrete speed limit levels.
        /// An in-game speed limit of 2.0 (e.g. on highway) is hereby translated into a discrete speed limit value of 100 (km/h).
        /// </summary>
        /// <param name="laneId"></param>
        /// <returns></returns>
        public float GetCustomSpeedLimit(uint laneId) {
			// check custom speed limit
			var setSpeedLimit = Flags.getLaneSpeedLimit(laneId);
			if (setSpeedLimit != null) {
				return setSpeedLimit.Value;
			}

			// check default speed limit
			ushort segmentId = Singleton<NetManager>.instance.m_lanes.m_buffer[laneId].m_segment;
			if (!MayHaveCustomSpeedLimits(segmentId, ref Singleton<NetManager>.instance.m_segments.m_buffer[segmentId])) {
				return 0;
			}
			
			var segmentInfo = Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].Info;

			uint curLaneId = Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].m_lanes;
			int laneIndex = 0;
			while (laneIndex < segmentInfo.m_lanes.Length && curLaneId != 0u) {
				if (curLaneId == laneId) {
					NetInfo.Lane laneInfo = segmentInfo.m_lanes[laneIndex];
					if (!MayHaveCustomSpeedLimits(laneInfo))
						return 0;

					return laneInfo.m_speedLimit;
				}

				laneIndex++;
				curLaneId = Singleton<NetManager>.instance.m_lanes.m_buffer[curLaneId].m_nextLane;
			}

			Log.Warning($"Speed limit for lane {laneId} could not be determined.");
			return 0; // no speed limit found
		}

		/// <summary>
		/// Determines the currently set speed limit for the given lane in terms of game (floating point) speed limit levels
		/// </summary>
		/// <param name="laneId"></param>
		/// <returns></returns>
		public float GetGameSpeedLimit(uint laneId) {
			return ToGameSpeedLimit(GetCustomSpeedLimit(laneId));
		}

		public float GetLockFreeGameSpeedLimit(ushort segmentId, byte laneIndex, uint laneId, NetInfo.Lane laneInfo) {
			if (! Options.customSpeedLimitsEnabled || ! MayHaveCustomSpeedLimits(laneInfo)) {
				return laneInfo.m_speedLimit;
			}

			float speedLimit = 0;
			float?[] fastArray = Flags.laneSpeedLimitArray[segmentId];
			if (fastArray != null && fastArray.Length > laneIndex && fastArray[laneIndex] != null) {
				speedLimit = ToGameSpeedLimit((float)fastArray[laneIndex]);
			} else {
				speedLimit = laneInfo.m_speedLimit;
			}
			return speedLimit;
		}

		/// <summary>
		/// Converts a custom speed limit to a game speed limit.
		/// </summary>
		/// <param name="customSpeedLimit"></param>
		/// <returns></returns>
		public float ToGameSpeedLimit(float customSpeedLimit) {
			return SpeedLimit.IsZero(customSpeedLimit)
				       ? MAX_SPEED
				       : customSpeedLimit;
		}

		/// <summary>
		/// Explicitly stores currently set speed limits for all segments of the specified NetInfo
		/// </summary>
		/// <param name="info"></param>
		public void FixCurrentSpeedLimits(NetInfo info) {
			if (info == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.FixCurrentSpeedLimits: info is null!");
#endif
				return;
			}

			if (info.name == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.FixCurrentSpeedLimits: info.name is null!");
#endif
				return;
			}
			if (!customizableNetInfos.Contains(info)) {
				return;
			}
			for (uint laneId = 1; laneId < NetManager.MAX_LANE_COUNT; ++laneId) {
				if (!Services.NetService.IsLaneValid(laneId)) {
					continue;
				}
				var segmentId = Singleton<NetManager>.instance.m_lanes.m_buffer[laneId].m_segment;
				var laneInfo = Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].Info;
				if (laneInfo.name != info.name
				    && (!childNetInfoNamesByCustomizableNetInfoName.ContainsKey(info.name)
				        || !childNetInfoNamesByCustomizableNetInfoName[info.name].Contains(laneInfo.name))) {
					continue;
				}
				Flags.setLaneSpeedLimit(laneId, GetCustomSpeedLimit(laneId));
			}
		}

		/// <summary>
		/// Explicitly clear currently set speed limits for all segments of the specified NetInfo
		/// </summary>
		/// <param name="info"></param>
		public void ClearCurrentSpeedLimits(NetInfo info) {
			if (info == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.ClearCurrentSpeedLimits: info is null!");
#endif
				return;
			}

			if (info.name == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.ClearCurrentSpeedLimits: info.name is null!");
#endif
				return;
			}

			if (!customizableNetInfos.Contains(info))
				return;

			for (uint laneId = 1; laneId < NetManager.MAX_LANE_COUNT; ++laneId) {
				if (!Services.NetService.IsLaneValid(laneId))
					continue;

				NetInfo laneInfo = Singleton<NetManager>.instance.m_segments.m_buffer[Singleton<NetManager>.instance.m_lanes.m_buffer[laneId].m_segment].Info;
				if (laneInfo.name != info.name && (!childNetInfoNamesByCustomizableNetInfoName.ContainsKey(info.name) || !childNetInfoNamesByCustomizableNetInfoName[info.name].Contains(laneInfo.name)))
					continue;

				Flags.removeLaneSpeedLimit(laneId);
			}
		}

		/// <summary>
		/// Determines the game default speed limit of the given NetInfo.
		/// </summary>
		/// <param name="info">the NetInfo of which the game default speed limit should be determined</param>
		/// <param name="roundToSignLimits">if true, custom speed limit are rounded to speed limits available as speed limit sign</param>
		/// <returns></returns>
		public float GetVanillaNetInfoSpeedLimit(NetInfo info, bool roundToSignLimits = true) {
			if (info == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.GetVanillaNetInfoSpeedLimit: info is null!");
#endif
				return 0;
			}

			if (info.m_netAI == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.GetVanillaNetInfoSpeedLimit: info.m_netAI is null!");
#endif
				return 0;
			}

			if (info.name == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.GetVanillaNetInfoSpeedLimit: info.name is null!");
#endif
				return 0;
			}

			string infoName = info.name;
			float[] vanillaSpeedLimits;
			if (!vanillaLaneSpeedLimitsByNetInfoName.TryGetValue(infoName, out vanillaSpeedLimits)) {
				return 0;
			}

			float? maxSpeedLimit = null;
			foreach (float speedLimit in vanillaSpeedLimits) {
				if (maxSpeedLimit == null || speedLimit > maxSpeedLimit) {
					maxSpeedLimit = speedLimit;
				}
			}

			return maxSpeedLimit ?? 0;
		}

		/// <summary>
		/// Determines the custom speed limit of the given NetInfo.
		/// </summary>
		/// <param name="info">the NetInfo of which the custom speed limit should be determined</param>
		/// <returns>-1 if no custom speed limit was set</returns>
		public float GetCustomNetInfoSpeedLimit(NetInfo info) {
			if (info == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.SetCustomNetInfoSpeedLimitIndex: info is null!");
#endif
				return -1;
			}

			if (info.name == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.SetCustomNetInfoSpeedLimitIndex: info.name is null!");
#endif
				return -1;
			}

			var infoName = info.name;
			float speedLimit;
			return !CustomLaneSpeedLimitByNetInfoName.TryGetValue(infoName, out speedLimit)
				       ? GetVanillaNetInfoSpeedLimit(info, true)
				       : speedLimit;
		}

		/// <summary>
		/// Sets the custom speed limit of the given NetInfo.
		/// </summary>
		/// <param name="info">the NetInfo for which the custom speed limit should be set</param>
		/// <param name="customSpeedLimit">The speed value to set in game speed units</param>
		public void SetCustomNetInfoSpeedLimit(NetInfo info, float customSpeedLimit) {
			if (info == null) {
#if DEBUG
				Log.Warning($"SetCustomNetInfoSpeedLimitIndex: info is null!");
#endif
				return;
			}

			if (info.name == null) {
#if DEBUG
				Log.Warning($"SetCustomNetInfoSpeedLimitIndex: info.name is null!");
#endif
				return;
			}

			string infoName = info.name;
			CustomLaneSpeedLimitByNetInfoName[infoName] = customSpeedLimit;

			float gameSpeedLimit = ToGameSpeedLimit(customSpeedLimit);

			// save speed limit in all NetInfos
			Log._Debug($"Updating parent NetInfo {infoName}: Setting speed limit to {gameSpeedLimit}");
			UpdateNetInfoGameSpeedLimit(info, gameSpeedLimit);

			List<string> childNetInfoNames;
			if (childNetInfoNamesByCustomizableNetInfoName.TryGetValue(infoName, out childNetInfoNames)) {
				foreach (var childNetInfoName in childNetInfoNames) {
					NetInfo childNetInfo;
					if (NetInfoByName.TryGetValue(childNetInfoName, out childNetInfo)) {
						Log._Debug($"Updating child NetInfo {childNetInfoName}: Setting speed limit to {gameSpeedLimit}");
						CustomLaneSpeedLimitByNetInfoName[childNetInfoName] = customSpeedLimit;
						UpdateNetInfoGameSpeedLimit(childNetInfo, gameSpeedLimit);
					}
				}
			}
		}

		private void UpdateNetInfoGameSpeedLimit(NetInfo info, float gameSpeedLimit) {
			if (info == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.UpdateNetInfoGameSpeedLimit: info is null!");
#endif
				return;
			}

			if (info.name == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.UpdateNetInfoGameSpeedLimit: info.name is null!");
#endif
				return;
			}

			if (info.m_lanes == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.UpdateNetInfoGameSpeedLimit: info.name is null!");
#endif
				return;
			}

			Log._Debug($"Updating speed limit of NetInfo {info.name} to {gameSpeedLimit}");

			foreach (NetInfo.Lane lane in info.m_lanes) {
				// TODO refactor check
				if ((lane.m_vehicleType & VEHICLE_TYPES) != VehicleInfo.VehicleType.None) {
					lane.m_speedLimit = gameSpeedLimit;
				}
			}
		}

		/// <summary>Sets the speed limit of a given lane.</summary>
		/// <param name="segmentId"></param>
		/// <param name="laneIndex"></param>
		/// <param name="laneInfo"></param>
		/// <param name="laneId"></param>
		/// <param name="speedLimit">Game speed units, 0=unlimited</param>
		/// <returns></returns>
		public bool SetSpeedLimit(ushort segmentId, uint laneIndex, NetInfo.Lane laneInfo, uint laneId, float speedLimit) {
			if (!MayHaveCustomSpeedLimits(laneInfo)) {
				return false;
			}
			if (!SpeedLimit.IsValidRange(speedLimit)) {
				return false;
			}
			if (!Services.NetService.IsLaneValid(laneId)) {
				return false;
			}

			Flags.setLaneSpeedLimit(segmentId, laneIndex, laneId, speedLimit);
			return true;
		}

		/// <summary>
		/// Sets the speed limit of a given segment and lane direction.
		/// </summary>
		/// <param name="segmentId"></param>
		/// <param name="finalDir"></param>
		/// <param name="speedLimit"></param>
		/// <returns></returns>
		public bool SetSpeedLimit(ushort segmentId, NetInfo.Direction finalDir, float speedLimit) {
			if (!MayHaveCustomSpeedLimits(segmentId, ref Singleton<NetManager>.instance.m_segments.m_buffer[segmentId])) {
				return false;
			}
			if (!SpeedLimit.IsValidRange(speedLimit)) {
				return false;
			}

			var segmentInfo = Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].Info;

			if (segmentInfo == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.SetSpeedLimit: info is null!");
#endif
				return false;
			}

			if (segmentInfo.m_lanes == null) {
#if DEBUG
				Log.Warning($"SpeedLimitManager.SetSpeedLimit: info.name is null!");
#endif
				return false;
			}

			uint curLaneId = Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].m_lanes;
			int laneIndex = 0;
			while (laneIndex < segmentInfo.m_lanes.Length && curLaneId != 0u) {
				NetInfo.Lane laneInfo = segmentInfo.m_lanes[laneIndex];
				NetInfo.Direction d = laneInfo.m_finalDirection;
				if (d != finalDir) {
					goto nextIter;
				}
				if (!MayHaveCustomSpeedLimits(laneInfo)) {
					goto nextIter;
				}
#if DEBUG
				Log._Debug($"SpeedLimitManager: Setting speed limit of lane {curLaneId} " +
				           $"to {speedLimit * SpeedLimit.SPEED_TO_KMPH}");
#endif
				Flags.setLaneSpeedLimit(curLaneId, speedLimit);

			nextIter:
				curLaneId = Singleton<NetManager>.instance.m_lanes.m_buffer[curLaneId].m_nextLane;
				laneIndex++;
			}

			return true;
		}

		public List<NetInfo> GetCustomizableNetInfos() {
			return customizableNetInfos;
		}

		public override void OnBeforeLoadData() {
			base.OnBeforeLoadData();

			// determine vanilla speed limits and customizable NetInfos
			SteamHelper.DLC_BitMask dlcMask = SteamHelper.GetOwnedDLCMask();

			int numLoaded = PrefabCollection<NetInfo>.LoadedCount();

			vanillaLaneSpeedLimitsByNetInfoName.Clear();
			customizableNetInfos.Clear();
			CustomLaneSpeedLimitByNetInfoName.Clear();
			childNetInfoNamesByCustomizableNetInfoName.Clear();
			NetInfoByName.Clear();

			List<NetInfo> mainNetInfos = new List<NetInfo>();

			Log.Info($"SpeedLimitManager.OnBeforeLoadData: {numLoaded} NetInfos loaded.");
			for (uint i = 0; i < numLoaded; ++i) {
				NetInfo info = PrefabCollection<NetInfo>.GetLoaded(i);

				if (info == null || info.m_netAI == null || !(info.m_netAI is RoadBaseAI || info.m_netAI is MetroTrackAI || info.m_netAI is TrainTrackBaseAI) || !(info.m_dlcRequired == 0 || (uint)(info.m_dlcRequired & dlcMask) != 0u)) {
					if (info == null)
						Log.Warning($"SpeedLimitManager.OnBeforeLoadData: NetInfo @ {i} is null!");
					continue;
				}

				string infoName = info.name;
				if (infoName == null) {
					Log.Warning($"SpeedLimitManager.OnBeforeLoadData: NetInfo name @ {i} is null!");
					continue;
				}

				if (!vanillaLaneSpeedLimitsByNetInfoName.ContainsKey(infoName)) {
					if (info.m_lanes == null) {
						Log.Warning($"SpeedLimitManager.OnBeforeLoadData: NetInfo lanes @ {i} is null!");
						continue;
					}

					Log.Info($"Loaded road NetInfo: {infoName}");
					NetInfoByName[infoName] = info;
					mainNetInfos.Add(info);

					float[] vanillaLaneSpeedLimits = new float[info.m_lanes.Length];
					for (int k = 0; k < info.m_lanes.Length; ++k) {
						vanillaLaneSpeedLimits[k] = info.m_lanes[k].m_speedLimit;
					}
					vanillaLaneSpeedLimitsByNetInfoName[infoName] = vanillaLaneSpeedLimits;
				}
			}

			mainNetInfos.Sort(delegate(NetInfo a, NetInfo b) {
				bool aRoad = a.m_netAI is RoadBaseAI;
				bool bRoad = b.m_netAI is RoadBaseAI;

				if (aRoad != bRoad) {
					if (aRoad)
						return -1;
					else
						return 1;
				}

				bool aTrain = a.m_netAI is TrainTrackBaseAI;
				bool bTrain = b.m_netAI is TrainTrackBaseAI;

				if (aTrain != bTrain) {
					if (aTrain)
						return 1;
					else
						return -1;
				}

				bool aMetro = a.m_netAI is MetroTrackAI;
				bool bMetro = b.m_netAI is MetroTrackAI;

				if (aMetro != bMetro) {
					if (aMetro)
						return 1;
					else
						return -1;
				}

				if (aRoad && bRoad) {
					bool aHighway = ((RoadBaseAI)a.m_netAI).m_highwayRules;
					bool bHighway = ((RoadBaseAI)b.m_netAI).m_highwayRules;

					if (aHighway != bHighway) {
						if (aHighway)
							return 1;
						else
							return -1;
					}
				}

				int aNumVehicleLanes = 0;
				foreach (NetInfo.Lane lane in a.m_lanes) {
					if ((lane.m_laneType & LANE_TYPES) != NetInfo.LaneType.None)
						++aNumVehicleLanes;
				}

				int bNumVehicleLanes = 0;
				foreach (NetInfo.Lane lane in b.m_lanes) {
					if ((lane.m_laneType & LANE_TYPES) != NetInfo.LaneType.None)
						++bNumVehicleLanes;
				}

				int res = aNumVehicleLanes.CompareTo(bNumVehicleLanes);
				if (res == 0) {
					return a.name.CompareTo(b.name);
				} else {
					return res;
				}
			});

			// identify parent NetInfos
			int x = 0;
			while (x < mainNetInfos.Count) {
				NetInfo info = mainNetInfos[x];
				string infoName = info.name;

				// find parent with prefix name

				bool foundParent = false;
				for (int y = 0; y < mainNetInfos.Count; ++y) {
					NetInfo parentInfo = mainNetInfos[y];

					if (info.m_placementStyle == ItemClass.Placement.Procedural && !infoName.Equals(parentInfo.name) && infoName.StartsWith(parentInfo.name)) {
						Log.Info($"Identified child NetInfo {infoName} of parent {parentInfo.name}");
						List<string> childNetInfoNames;
						if (!childNetInfoNamesByCustomizableNetInfoName.TryGetValue(parentInfo.name, out childNetInfoNames)) {
							childNetInfoNamesByCustomizableNetInfoName[parentInfo.name] = childNetInfoNames = new List<string>();
						}
						childNetInfoNames.Add(info.name);
						NetInfoByName[infoName] = info;
						foundParent = true;
						break;
					}
				}

				if (foundParent) {
					mainNetInfos.RemoveAt(x);
				} else {
					++x;
				}
			}

			customizableNetInfos = mainNetInfos;
		}

		protected override void HandleInvalidSegment(SegmentGeometry geometry) {
			NetInfo segmentInfo = Singleton<NetManager>.instance.m_segments.m_buffer[geometry.SegmentId].Info;
			uint curLaneId = Singleton<NetManager>.instance.m_segments.m_buffer[geometry.SegmentId].m_lanes;
			int laneIndex = 0;
			while (laneIndex < segmentInfo.m_lanes.Length && curLaneId != 0u) {
				// NetInfo.Lane laneInfo = segmentInfo.m_lanes[laneIndex];
				// float? setSpeedLimit = Flags.getLaneSpeedLimit(curLaneId);

				Flags.setLaneSpeedLimit(curLaneId, null);

				curLaneId = Singleton<NetManager>.instance.m_lanes.m_buffer[curLaneId].m_nextLane;
				laneIndex++;
			}
		}

		protected override void HandleValidSegment(SegmentGeometry geometry) {

		}

		public bool LoadData(List<Configuration.LaneSpeedLimit> data) {
			bool success = true;
			Log.Info($"Loading lane speed limit data. {data.Count} elements");
			foreach (Configuration.LaneSpeedLimit laneSpeedLimit in data) {
				try {
					if (!Services.NetService.IsLaneValid(laneSpeedLimit.laneId)) {
						Log._Debug($"SpeedLimitManager.LoadData: Skipping lane {laneSpeedLimit.laneId}: " +
						           $"Lane is invalid");
						continue;
					}

					ushort segmentId = Singleton<NetManager>.instance.m_lanes.m_buffer[laneSpeedLimit.laneId].m_segment;
					NetInfo info = Singleton<NetManager>.instance.m_segments.m_buffer[segmentId].Info;
					var customSpeedLimit = GetCustomNetInfoSpeedLimit(info);
					Log._Debug($"SpeedLimitManager.LoadData: Handling lane {laneSpeedLimit.laneId}: " +
					           $"Custom speed limit of segment {segmentId} info ({info}, name={info?.name}, " +
					           $"lanes={info?.m_lanes} is {customSpeedLimit}");

					if (SpeedLimit.IsValidRange(customSpeedLimit)) {
						// lane speed limit differs from default speed limit
						Log._Debug($"SpeedLimitManager.LoadData: Loading lane speed limit: " +
						           $"lane {laneSpeedLimit.laneId} = {laneSpeedLimit.speedLimit} km/h");
						var kmph = laneSpeedLimit.speedLimit / SpeedLimit.SPEED_TO_KMPH; // convert to game units
						Flags.setLaneSpeedLimit(laneSpeedLimit.laneId, kmph);
					} else {
						Log._Debug($"SpeedLimitManager.LoadData: " +
						           $"Skipping lane speed limit of lane {laneSpeedLimit.laneId} " +
						           $"({laneSpeedLimit.speedLimit} km/h)");
					}
				} catch (Exception e) {
					// ignore, as it's probably corrupt save data. it'll be culled on next save
					Log.Warning("SpeedLimitManager.LoadData: Error loading speed limits: " + e.ToString());
					success = false;
				}
			}
			return success;
		}

		List<Configuration.LaneSpeedLimit> ICustomDataManager<List<Configuration.LaneSpeedLimit>>.SaveData(ref bool success) {
			var ret = new List<Configuration.LaneSpeedLimit>();
			foreach (var e in Flags.getAllLaneSpeedLimits()) {
				try {
					var laneSpeedLimit = new Configuration.LaneSpeedLimit(e.Key, e.Value);
					Log._Debug($"Saving speed limit of lane {laneSpeedLimit.laneId}: " +
					           $"{laneSpeedLimit.speedLimit*SpeedLimit.SPEED_TO_KMPH} km/h");
					ret.Add(laneSpeedLimit);
				} catch (Exception ex) {
					Log.Error($"Exception occurred while saving lane speed limit @ {e.Key}: {ex}");
					success = false;
				}
			}
			return ret;
		}

		public bool LoadData(Dictionary<string, float> data) {
			Log.Info($"Loading custom default speed limit data. {data.Count} elements");
			foreach (var e in data) {
				NetInfo netInfo;
				if (!NetInfoByName.TryGetValue(e.Key, out netInfo)) {
					continue;
				}

				if (e.Value >= 0f) {
					SetCustomNetInfoSpeedLimit(netInfo, e.Value);
				}
			}
			return true; // true = success
		}

		Dictionary<string, float> ICustomDataManager<Dictionary<string, float>>.SaveData(ref bool success) {
			var ret = new Dictionary<string, float>();
			foreach (var e in CustomLaneSpeedLimitByNetInfoName) {
				try {
					float gameSpeedLimit = ToGameSpeedLimit(e.Value);

					ret.Add(e.Key, gameSpeedLimit);
				} catch (Exception ex) {
					Log.Error($"Exception occurred while saving custom default speed limits @ {e.Key}: {ex.ToString()}");
					success = false;
				}
			}
			return ret;
		}

#if DEBUG
		/*public Dictionary<NetInfo, ushort> GetDefaultSpeedLimits() {
			Dictionary<NetInfo, ushort> ret = new Dictionary<NetInfo, ushort>();
			int numLoaded = PrefabCollection<NetInfo>.LoadedCount();
			for (uint i = 0; i < numLoaded; ++i) {
				NetInfo info = PrefabCollection<NetInfo>.GetLoaded(i);
				ushort defaultSpeedLimit = GetAverageDefaultCustomSpeedLimit(info, NetInfo.Direction.Forward);
				ret.Add(info, defaultSpeedLimit);
				Log._Debug($"Loaded NetInfo: {info.name}, placementStyle={info.m_placementStyle}, availableIn={info.m_availableIn}, thumbnail={info.m_Thumbnail} connectionClass.service: {info.GetConnectionClass().m_service.ToString()}, connectionClass.subService: {info.GetConnectionClass().m_subService.ToString()}, avg. default speed limit: {defaultSpeedLimit}");
			}
			return ret;
		}*/
#endif
	}
}
