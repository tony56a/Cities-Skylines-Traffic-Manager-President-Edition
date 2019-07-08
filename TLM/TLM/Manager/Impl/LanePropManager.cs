using ColossalFramework.Math;
using CSUtil.Commons;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading;
using TrafficManager.Traffic.Data;
using TrafficManager.UI;
using TrafficManager.Util;
using UnityEngine;

namespace TrafficManager.Manager.Impl {

		public class LanePropManager : AbstractCustomManager, ILanePropManager, IPreLoadManager {
				
				public static readonly LanePropManager Instance = new LanePropManager();

				/// <summary>
				/// The font used for rendering the 
				/// </summary>
				private Font signFont;

				private GameObject gameObj;

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
						{ "30", "UKR-S Blank" },
						{ "40", "UKR-S Blank" },
						{ "50", "UKR-S Blank" },
						{ "60", "UKR-S Blank" },
						{ "100", "UKR-S Blank" },
						{ "23", "speed limit 15" },
						{ "39", "speed limit 25" },
						{ "47", "speed limit 30" },
						{ "70", "speed limit 45" },
						{ "101", "speed limit 65" },
				};

				private LanePropManager() {
						speedLimitProps = new Dictionary<string, Dictionary<string, PropInfo>>();
				}

				/// <summary>
				/// Replace the Lane Prop, based on the original NetLane prop, as well asthe current segment/lane configurations
				/// </summary>
				/// <param name="originalNetLaneProp">The original Netlane prop to be rendered</param>
				/// <param name="segmentId">The segment the prop is on</param>
				/// <param name="laneId">The lane the prop is on</param>
				/// <returns>A new PropInfo to render, or the original if no replacement is available</returns>
				public PropInfo ReplaceProp(NetLaneProps.Prop originalNetLaneProp, ushort segmentId, uint laneId) {

						// Replace speed limit signs
						if (originalNetLaneProp.m_finalProp != null &&
								originalNetLaneProp.m_finalProp.name.ToLower().Contains("speed limit")) {
								NetInfo.Direction direction = ( originalNetLaneProp.m_angle == 0 ) ? NetInfo.Direction.Forward : NetInfo.Direction.Backward;
								float segmentSpeed = SpeedLimitManager.Instance.GetCustomSpeedLimit(segmentId, direction);
								string speedSignKey = SpeedLimit.ToKmphPrecise(segmentSpeed).ToString();
								if (speedLimitProps[FALLBACK_PROP_COLLECTION_KEY_NAME].ContainsKey(speedSignKey)) {


										return speedLimitProps[FALLBACK_PROP_COLLECTION_KEY_NAME][speedSignKey];
								}
								else {
										return originalNetLaneProp.m_finalProp;
								}
						}
						else {
								return originalNetLaneProp.m_finalProp;
						}
				}

				public override void OnLevelLoading() {
						List<PrefabInfo> allPropInfos = Resources.FindObjectsOfTypeAll<PrefabInfo>().Where(prefabInfo =>
																	prefabInfo.GetType().Equals(typeof(PropInfo))).ToList();
						//LoadProps(FALLBACK_SPEED_LIMIT_SIGN_PROPS, FALLBACK_PROP_COLLECTION_KEY_NAME, ref speedLimitProps, allPropInfos);
						LoadFallbackProps();

#if DEBUG
						PrintDebugInfo();
#endif
				}

				public void PreLoadData() {
						Font font = Font.CreateDynamicFontFromOSFont("Arial", 60);
						font.RequestCharactersInTexture("ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890", 60);
						var fakeObj = new GameObject("Fake faking dynamicsign");
						var fakeTextMesh = fakeObj.AddComponent<TextMesh>();
						fakeTextMesh.font = font;
						fakeTextMesh.GetComponent<Renderer>().material = font.material;
						fakeTextMesh.fontSize = 60;
						fakeTextMesh.text = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";

						Material fontMat = font.material;
						fontMat.color = Color.black;
						font.material.mainTexture.MakeReadable();
						signFont = font;
						gameObj = fakeObj;
						gameObj.SetActive(false);
				}



				/// <summary>
				/// Dump all of the props currently loaded in the manager
				/// </summary>
				protected override void InternalPrintDebugInfo() {

						IEnumerable<NetInfo> netInfos = Resources.FindObjectsOfTypeAll<NetInfo>().Where(x => ( x.m_hasForwardVehicleLanes || x.m_hasBackwardVehicleLanes ) && x != null);

						try {
								foreach (NetInfo netInfo in netInfos) {
										Log._Debug($"NetInfo: {netInfo}");
										foreach (NetInfo.Lane lane in netInfo.m_lanes) {
												Log._Debug($" - NetLane Type: {lane?.m_laneType.ToString()} Direction: {lane?.m_finalDirection}");
												foreach (NetLaneProps.Prop prop in lane?.m_laneProps?.m_props) {
														Log._Debug($"  -- NetLaneProp: {prop?.m_finalProp?.name}");
												}
										}
								}
						}
						catch (NullReferenceException e) {
								// do nothing, this is just debug logging
						}

						foreach (string collectionName in this.speedLimitProps.Keys) {
								Log._Debug($"Speed Limit Prop Collection: {collectionName}");
								foreach (KeyValuePair<string, PropInfo> entry in speedLimitProps[collectionName]) {
										Log._Debug($"{collectionName} prop name: {entry.Key}: {entry.Value.name}");
								}
						}

				}

				/// <summary>
				/// Find/Generate any fallback props to render when no user-provided props are available
				/// </summary>
				private void LoadFallbackProps() {
						// Generate speed limit fallback props
						var allPropInfos = Resources.FindObjectsOfTypeAll<PrefabInfo>().Where(prefabInfo =>
																	prefabInfo.GetType().Equals(typeof(PropInfo)));
						PropInfo originalProp = null;
						foreach(var propInfo in allPropInfos) {
								if (propInfo.name.ToLower().Contains("UKR-S Blank".ToLower())) {
										originalProp = propInfo as PropInfo;
								}
						}
									
						if (originalProp == null || originalProp.m_material == null) {
								Log._Debug("Cannot find fallback prop");
								return;
						}
						Dictionary<string, PropInfo> propCollectionToAdd = new Dictionary<string, PropInfo>();
						for (int i = 5; i <= 140; i += 5) {
								var speedString = i.ToString();
								var propName = $"TMPE sign {speedString}";
								GameObject signObj = GameObject.Instantiate<GameObject>(originalProp.gameObject);
								signObj.SetActive(true);
								signObj.name = propName;
								PropInfo replacementProp = signObj.GetComponent<PropInfo>();
								MeshRenderer meshRenderer = signObj.GetComponent<MeshRenderer>();
								Material material = GameObject.Instantiate<Material>(originalProp.m_material);

								var newTexture = new Texture2D(material.mainTexture.width, material.mainTexture.height, TextureFormat.ARGB32, false);
								newTexture.SetPixels(((Texture2D)material.mainTexture).GetPixels());

								TextureUtil.DrawText(ref newTexture, speedString, signFont, 953, 432, 60);
								material.SetTexture("_MainTex", newTexture);
								meshRenderer.material = material;
								PrefabCollection<PropInfo>.InitializePrefabs("Custom Assets", replacementProp, null);
								replacementProp.m_lodRenderDistance = originalProp.m_lodRenderDistance * 2;
								replacementProp.m_maxRenderDistance = 20000;

								if (replacementProp != null) {
										propCollectionToAdd.Add(i.ToString(), replacementProp);
								}

						}

						speedLimitProps.Add(FALLBACK_PROP_COLLECTION_KEY_NAME, propCollectionToAdd);
				}

				/// <summary>
				/// Loads the props for a given prop collection and prop type(speed limit signs, highway signs, etc)
				/// </summary>
				/// <param name="propNames">The dictionary holding the prop names for the collection</param>
				/// <param name="propCollectionKeyName">The name of the prop collection being loaded</param>
				/// <param name="propCollections">The dictionary holding all prop collections of the same type</param>
				/// <param name="propInfos">All props currently loaded in the game</param>
				private void LoadProps(Dictionary<string, string> propNames, string propCollectionKeyName, ref Dictionary<string, Dictionary<string, PropInfo>> propCollections, List<PropInfo> propInfos) {
						Dictionary<string, PropInfo> propCollectionToAdd = new Dictionary<string, PropInfo>();
						foreach (KeyValuePair<string, string> entry in propNames) {
								foreach (PropInfo prefab in propInfos) {
										if (prefab.name.ToLower().Contains(entry.Value.ToLower())) {
												propCollectionToAdd[entry.Key] = prefab as PropInfo;
										}
								}
						}

						propCollections.Add(propCollectionKeyName, propCollectionToAdd);
				}


		}
}
