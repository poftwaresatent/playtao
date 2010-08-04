/*
 * Play TAO --- a tool for the Stanford Whole-Body Control Framework
 *              http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file gfx/viewport.cpp
   \author Roland Philippsen
*/

#include "viewport.hpp"
#include "wrap_glu.hpp"
#include <limits>
#include <cmath>

using namespace std;

namespace gfx {

  
  Viewport::
  Viewport()
  {
    UpdateSubwin(0, 0, 1, 1);
    ResetShape();
    ResetBounds();
  }
  
  
  Viewport::
  Viewport(double subwin_x0, double subwin_y0, double subwin_x1, double subwin_y1)
  {
    UpdateSubwin(subwin_x0, subwin_y0, subwin_x1, subwin_y1);
    ResetShape();
    ResetBounds();
  }
  
  
  void Viewport::
  UpdateSubwin(double x0, double y0, double x1, double y1)
  {
    subwin_.x0 = x0;
    subwin_.y0 = y0;
    subwin_.width = x1 - x0;
    subwin_.height = y1 - y0;
    dirty_ = true;
  }
  
  
  void Viewport::
  UpdateShape(int width, int height)
  {
    shape_.winwidth = width;
    shape_.winheight = height;
    dirty_ = true;
  }
  
  
  void Viewport::
  ResetShape()
  {
    shape_.winwidth = -1;
    dirty_ = false;
  }
  
  
  void Viewport::
  UpdateBounds(double xx, double yy, double zz)
  {
    if (xx < bounds_.x0) {
      bounds_.x0 = xx;
      dirty_ = true;
    }
    if (xx > bounds_.x1) {
      bounds_.x1 = xx;
      dirty_ = true;
    }
    if (yy < bounds_.y0) {
      bounds_.y0 = yy;
      dirty_ = true;
    }
    if (yy > bounds_.y1) {
      bounds_.y1 = yy;
      dirty_ = true;
    }
    if (zz < bounds_.z0) {
      bounds_.z0 = zz;
      dirty_ = true;
    }
    if (zz > bounds_.z1) {
      bounds_.z1 = zz;
      dirty_ = true;
    }
  }
  
  
  void Viewport::
  ResetBounds()
  {
    bounds_.x0 = numeric_limits<double>::max();
    bounds_.x1 = numeric_limits<double>::min();
    dirty_ = false;
  }
  
  
  void Viewport::
  PushOrtho(double buffer_distance)
  {
    MaybeUpdate();
    
    glViewport(padding_.x0, padding_.y0, padding_.width, padding_.height);
    gluLookAt(eye_.x, eye_.y, eye_.z,
	      sphere_.cx, sphere_.cy, sphere_.cz,
	      0, 1, 0);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(frustum_.left - buffer_distance, frustum_.right + buffer_distance,
	    frustum_.bottom - buffer_distance, frustum_.top + buffer_distance,
	    frustum_.near - buffer_distance, frustum_.far + buffer_distance);
  }
  
  
  void Viewport::
  Pop()
  {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();  
  }
  
  
  void Viewport::
  UpdateSphere()
  {
    if (bounds_.x0 > bounds_.x1) { // not ready
      sphere_.cx = 0;
      sphere_.cy = 0;
      sphere_.cz = 0;
      sphere_.radius = 1;
    }
    else {
      sphere_.cx = (bounds_.x0 + bounds_.x1) / 2;
      sphere_.cy = (bounds_.y0 + bounds_.y1) / 2;
      sphere_.cz = (bounds_.z0 + bounds_.z1) / 2;
      sphere_.radius = sqrt(pow(bounds_.x1 - sphere_.cx, 2)
			    + pow(bounds_.y1 - sphere_.cy, 2)
			    + pow(bounds_.z1 - sphere_.cz, 2));
    }
  }
  
  
  void Viewport::
  UpdateOrtho()
  {
    frustum_.left   = -sphere_.radius;
    frustum_.right  =  sphere_.radius;
    frustum_.bottom = -sphere_.radius;
    frustum_.top    =  sphere_.radius;
    frustum_.near   = -sphere_.radius;
    frustum_.far    =  sphere_.radius;
  }
  
  
  void Viewport::
  UpdatePadding()
  {
    if (0 > shape_.winwidth) {	// not ready
      padding_.x0 = 0;
      padding_.y0 = 0;
      padding_.width = 1;
      padding_.height = 1;
    }
    else {
      shape_.x0 = ceil(shape_.winwidth * subwin_.x0);
      shape_.y0 = ceil(shape_.winheight * subwin_.y0);
      shape_.width = floor(shape_.winwidth * subwin_.width);
      shape_.height = floor(shape_.winheight * subwin_.height);
      
      int const pad((shape_.width - shape_.height) / 2);
      if (pad > 0) {		// pad left and right
	padding_.x0 = shape_.x0 + pad;
	padding_.y0 = shape_.y0;
	padding_.width = shape_.height;
	padding_.height = shape_.height;
      }
      else if (pad < 0) {	// pad below and above
	padding_.x0 = shape_.x0;
	padding_.y0 = shape_.y0 - pad; // pad<0 so to add it we need -
	padding_.width = shape_.width;
	padding_.height = shape_.width;
      }
      else {			// no padding
	padding_.x0 = shape_.x0;
	padding_.y0 = shape_.y0;
	padding_.width = shape_.width;
	padding_.height = shape_.height;
      }
    }
  }
  
  
  void Viewport::
  UpdateEye(double distance)
  {
    eye_.x = sphere_.cx;
    eye_.y = sphere_.cy;
    eye_.z = sphere_.cz - sphere_.radius - distance;
  }
  
  
  double Viewport::
  GetRadius()
  {
    MaybeUpdate();
    return sphere_.radius;
  }
  
  
  void Viewport::
  MaybeUpdate()
  {
    if (dirty_) {
      UpdateSphere();
      UpdateOrtho();
      UpdatePadding();
      UpdateEye(sphere_.radius);
      dirty_ = false;
    }
  }
  
}
