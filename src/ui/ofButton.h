#pragma once

/**
 * UI Button that can be clicked
 */
struct ofButton {
  ofImage data;
  int dx, dy;
  bool hover, state;

  void draw(){
      data.draw(ofGetViewportWidth() - dx, dy);
  }

  bool contains(int mx, int my) {
      int x = ofGetViewportWidth() - dx;
      int y = dy;
      return mx >= x && mx < x + data.getWidth()
          && my >= y && my < y + data.getHeight();
  }
};
