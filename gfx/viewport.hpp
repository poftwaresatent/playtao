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
   \file gfx/viewport.hpp
   \author Roland Philippsen
*/

#ifndef GFX_VIEWPORT_HPP
#define GFX_VIEWPORT_HPP

namespace gfx {
  
  class Viewport
  {
  public:
    Viewport();
    
    void UpdateShape(int width, int height);
    void ResetShape();
    void UpdateBounds(double xx, double yy, double zz);
    void ResetBounds();
    void PushOrtho(double buffer_distance);
    void Pop();
    double GetRadius();
    
  protected:
    void MaybeUpdate();
    void UpdateSphere();
    void UpdateOrtho();
    void UpdatePadding();
    void UpdateEye(double distance);
    
    struct {
      double x0, x1, y0, y1, z0, z1;
    } bounds_;
    
    struct {
      int width, height;
    } shape_;
    
    struct {
      double cx, cy, cz, radius;
    } sphere_;
    
    struct {
      int x0, y0, width, height;
    } padding_;
    
    struct {
      double left, right, bottom, top, near, far;
    } frustum_;
    
    struct {
      double x, y, z;
    } eye_;
    
    bool dirty_;
  };

}

#endif // GFX_VIEWPORT_HPP
