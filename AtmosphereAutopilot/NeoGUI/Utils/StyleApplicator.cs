/*
 * Atmosphere Autopilot, plugin for Kerbal Space Program.
 * 
 * Copyright (C) 2016, George Sedov.
 * 
 * Atmosphere Autopilot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * Atmosphere Autopilot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
 */

using UnityEngine;
using UnityEngine.UI;

namespace AtmosphereAutopilot.UI {
public class StyleApplicator : MonoBehaviour {
  public enum ElementTypes {
    None,
    Window,
    Button,
    ButtonToggle,
    Slider
  }

  [SerializeField]
  private ElementTypes m_ElementType = ElementTypes.None;

  public ElementTypes ElementType {
    get { return m_ElementType; }
  }

  public void SetImage(Sprite sprite, Image.Type type) {
    Image image = GetComponent<Image>();
    if (image == null)
      return;

    image.sprite = sprite;
    image.type = type;
  }

  public void SetSelectable(Sprite normal, Sprite highlight, Sprite pressed, Sprite disabled) {
    Selectable selectable = GetComponent<Selectable>();
    if (selectable != null) {
      selectable.image.sprite = normal;
      selectable.image.type = Image.Type.Sliced;
      selectable.transition = Selectable.Transition.SpriteSwap;
      SpriteState spriteState = selectable.spriteState;
      spriteState.highlightedSprite = highlight;
      spriteState.pressedSprite = pressed;
      spriteState.disabledSprite = disabled;
      selectable.spriteState = spriteState;
    }
  }

  public void SetToggle(Sprite normal, Sprite highlight, Sprite pressed, Sprite disabled) {
    SetSelectable(normal, highlight, pressed, disabled);

    Image toggleImage = GetComponent<Toggle>()?.graphic as Image;
    if (toggleImage != null) {
      toggleImage.sprite = pressed;
      toggleImage.type = Image.Type.Sliced;
    }
  }

  public void SetSlider (Sprite normal, Sprite highlight, Sprite pressed, Sprite disabled, Sprite background) {
    Slider slider = GetComponent<Slider>();
    if (slider != null) {
      slider.image.sprite = normal;
      slider.image.type = Image.Type.Sliced;
      slider.transition = Selectable.Transition.SpriteSwap;
      SpriteState spriteState = slider.spriteState;
      spriteState.highlightedSprite = highlight;
      spriteState.pressedSprite = pressed;
      spriteState.disabledSprite = disabled;
      slider.spriteState = spriteState;
    }
    SetImage (background, Image.Type.Sliced);
  }
}
}
