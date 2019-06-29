using CSUtil.Commons;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using TrafficManager.Traffic.Data;
using UnityEngine;

namespace TrafficManager.Manager.Impl
{
		public class LanePropManager : AbstractCustomManager, ILanePropManager
		{
				public static readonly LanePropManager Instance = new LanePropManager();

				/// <summary>
				/// All of the speed limit sign props
				/// </summary>
				private Dictionary<string, Dictionary<string, PropInfo>> speedLimitProps;

				/// <summary>
				/// The key name for the fallback(vanilla) prop collections
				/// </summary>
				private static readonly string FALLBACK_PROP_COLLECTION_KEY_NAME = "fallback";

				private static readonly Dictionary<string, string> FALLBACK_SPEED_LIMIT_SIGN_PROPS = new Dictionary<string, string>
				{
						{ "30", "30 speed limit" },
						{ "40", "40 speed limit" },
						{ "50", "50 speed limit" },
						{ "60", "60 speed limit" },
						{ "100", "100 speed limit" },
				};

				private LanePropManager()
				{
					speedLimitProps = new Dictionary<string, Dictionary<string, PropInfo>>();
				}

				/// <summary>
				/// Replace the Lane Prop, based on the original NetLane prop, as well asthe current segment/lane configurations
				/// </summary>
				/// <param name="originalNetLaneProp">The original Netlane prop to be rendered</param>
				/// <param name="segmentId">The segment the prop is on</param>
				/// <param name="laneId">The lane the prop is on</param>
				/// <returns>A new PropInfo to render, or the original if no replacement is available</returns>
				public PropInfo ReplaceProp(NetLaneProps.Prop originalNetLaneProp, ushort segmentId, uint laneId)
				{
						NetInfo.Direction direction = (originalNetLaneProp.m_angle == 0) ? NetInfo.Direction.Forward : NetInfo.Direction.Backward;
						float segmentSpeed = SpeedLimitManager.Instance.GetCustomSpeedLimit(segmentId, direction);
						string speedSignKey = SpeedLimit.ToKmphPrecise(segmentSpeed).ToString();

						if (originalNetLaneProp.m_finalProp != null && 
								originalNetLaneProp.m_finalProp.name.ToLower().Contains("speed limit") &&
								speedLimitProps[FALLBACK_PROP_COLLECTION_KEY_NAME].ContainsKey(speedSignKey))
						{
								return speedLimitProps[FALLBACK_PROP_COLLECTION_KEY_NAME][speedSignKey];
						}
						else
						{
								return originalNetLaneProp.m_finalProp;
						}
				}

				public override void OnAfterLoadData()
				{
						List<PrefabInfo> AllPropInfos = Resources.FindObjectsOfTypeAll<PrefabInfo>().Where(prefabInfo =>
											prefabInfo.GetType().Equals(typeof(PropInfo))).ToList();
						LoadProps(FALLBACK_SPEED_LIMIT_SIGN_PROPS, FALLBACK_PROP_COLLECTION_KEY_NAME, ref speedLimitProps, AllPropInfos);
#if DEBUG
						PrintDebugInfo();
#endif
				}

				/// <summary>
				/// Dump all of the props currently loaded in the manager
				/// </summary>
				protected override void InternalPrintDebugInfo()
				{
						base.InternalPrintDebugInfo();
						foreach (string collectionName in this.speedLimitProps.Keys)
						{
								Log._Debug($"Speed Limit Prop Collection: {collectionName}");
								foreach (KeyValuePair<string, PropInfo> entry in speedLimitProps[collectionName])
								{
										Log._Debug($"{collectionName} prop name: {entry.Key}: {entry.Value.name}");
								}
						}
						
				}

				/// <summary>
				/// Loads the props for a given prop collection and prop type(speed limit signs, highway signs, etc)
				/// </summary>
				/// <param name="propNames">The dictionary holding the prop names for the collection</param>
				/// <param name="propCollectionKeyName">The name of the prop collection being loaded</param>
				/// <param name="propCollections">The dictionary holding all prop collections of the same type</param>
				/// <param name="propInfos">All props currently loaded in the game</param>
				private void LoadProps(Dictionary<string, string> propNames, string propCollectionKeyName, ref Dictionary<string, Dictionary<string, PropInfo>> propCollections, List<PrefabInfo> propInfos)
				{
						Dictionary<string, PropInfo> propCollectionToAdd = new Dictionary<string, PropInfo>();
						foreach (KeyValuePair<string, string> entry in propNames)
						{
								foreach (PrefabInfo prefab in propInfos)
								{
										if (prefab.name.ToLower().Contains(entry.Value))
										{
												propCollectionToAdd[entry.Key] = prefab as PropInfo;
										}
								}
						}

						propCollections.Add(propCollectionKeyName, propCollectionToAdd);
				}


		}
}
