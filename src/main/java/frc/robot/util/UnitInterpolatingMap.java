/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2026 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class UnitInterpolatingMap<K extends Unit, V extends Unit> {
  private final InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
  private final K keyUnit;
  private final V valueUnit;

  public UnitInterpolatingMap(K keyUnit, V valueUnit) {
    this.keyUnit = keyUnit;
    this.valueUnit = valueUnit;
  }

  public void put(Measure<K> key, Measure<V> value) {
    map.put(key.in(keyUnit), value.in(valueUnit));
  }

  @SuppressWarnings("unchecked")
  public Measure<V> get(Measure<K> key) {
    return (Measure<V>) valueUnit.of(map.get(key.in(keyUnit)));
  }
}
