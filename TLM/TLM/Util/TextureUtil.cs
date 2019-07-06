using ColossalFramework.UI;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

namespace TrafficManager.Util {
	public static class TextureUtil {
		public static UITextureAtlas GenerateLinearAtlas(string name, Texture2D texture, int numSprites, string[] spriteNames) {
			return Generate2DAtlas(name, texture, numSprites, 1, spriteNames);
		}

		public static UITextureAtlas Generate2DAtlas(string name, Texture2D texture, int numX, int numY, string[] spriteNames) {
			if (spriteNames.Length != numX * numY) {
				throw new ArgumentException($"Number of sprite name does not match dimensions (expected {numX} x {numY}, was {spriteNames.Length})");
			}

			UITextureAtlas atlas = ScriptableObject.CreateInstance<UITextureAtlas>();
			atlas.padding = 0;
			atlas.name = name;

			var shader = Shader.Find("UI/Default UI Shader");
			if (shader != null)
				atlas.material = new Material(shader);
			atlas.material.mainTexture = texture;

			int spriteWidth = Mathf.RoundToInt((float)texture.width / (float)numX);
			int spriteHeight = Mathf.RoundToInt((float)texture.height / (float)numY);

			int k = 0;
			for (int i = 0; i < numX; ++i) {
				float x = (float)i / (float)numX;
				for (int j = 0; j < numY; ++j) {
					float y = (float)j / (float)numY;

					var sprite = new UITextureAtlas.SpriteInfo {
						name = spriteNames[k],
						region = new Rect(x, y, (float)spriteWidth / (float)texture.width, (float)spriteHeight / (float)texture.height)
					};

					var spriteTexture = new Texture2D(spriteWidth, spriteHeight);
					spriteTexture.SetPixels(texture.GetPixels((int)((float)texture.width * sprite.region.x), (int)((float)texture.height * sprite.region.y), spriteWidth, spriteHeight));
					sprite.texture = spriteTexture;

					atlas.AddSprite(sprite);

					++k;
				}
			}

			return atlas;
		}


		public static Texture2D DrawText(ref Texture2D tx, string sText, Font myFont, int startX, int startY, int size) {
						CharacterInfo ci;
						char[] cText = sText.ToCharArray();

						Material fontMat = myFont.material;
						Texture2D fontTx = fontMat.mainTexture as Texture2D;

						fontTx.filterMode = FilterMode.Point;
						RenderTexture rt = RenderTexture.GetTemporary(fontTx.width, fontTx.height);
						rt.filterMode = FilterMode.Point;
						RenderTexture.active = rt;
						Graphics.Blit(fontTx, rt);
						Texture2D img2 = new Texture2D(fontTx.width, fontTx.height);
						img2.ReadPixels(new Rect(0, 0, fontTx.width, fontTx.height), 0, 0);
						img2.Apply();
						RenderTexture.ReleaseTemporary(rt);
						fontTx = img2;

						int x, y, w, h;
						int posX = startX;

						for (int i = 0; i < cText.Length; i++)
						{
								myFont.GetCharacterInfo(cText[i], out ci, size);

								x = (int)((float)fontTx.width * ci.uv.x);
								y = (int)((float)fontTx.height * (ci.uv.y + ci.uv.height));
								w = (int)((float)fontTx.width * ci.uv.width);
								h = (int)((float)fontTx.height * (-ci.uv.height));

								Color[] cChar = fontTx.GetPixels(x, y, w, h);
								for (int row = 0; row < h; ++row)
								{
										Array.Reverse(cChar, row * w, w);
								}
								Array.Reverse(cChar);
								var blah = cChar.ToList().ConvertAll<Color32>(pixel => ( pixel.a > 0 ) ? Color.red : Color.white).ToArray();

								tx.SetPixels32(posX, startY, w, h, blah);
								tx.Apply();
								posX += ci.advance;
						}

						byte[] bytes = tx.EncodeToPNG();
						File.WriteAllBytes(Application.dataPath + $"/../{sText}.png", bytes);

						return tx;
				}
		}
}
