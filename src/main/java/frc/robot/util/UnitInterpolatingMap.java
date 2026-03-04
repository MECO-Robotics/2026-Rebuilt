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
import java.util.NavigableSet;
import java.util.TreeSet;

public class UnitInterpolatingMap<K extends Unit, V extends Unit> {
  private final InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
  private final NavigableSet<Double> keys = new TreeSet<>();
  private final K keyUnit;
  private final V valueUnit;

  public UnitInterpolatingMap(K keyUnit, V valueUnit) {
    this.keyUnit = keyUnit;
    this.valueUnit = valueUnit;
  }

  public void put(Measure<K> key, Measure<V> value) {
    var keyValue = key.in(keyUnit);
    map.put(keyValue, value.in(valueUnit));
    keys.add(keyValue);
  }

  @SuppressWarnings("unchecked")
  public Measure<V> get(Measure<K> key) {
    if (keys.isEmpty()) {
      return (Measure<V>) valueUnit.of(0.0);
    }

    var keyValue = key.in(keyUnit);
    var clampedKey = Math.max(keys.first(), Math.min(keys.last(), keyValue));
    var interpolated = map.get(clampedKey);
    if (interpolated == null) {
      return (Measure<V>) valueUnit.of(0.0);
    }

    return (Measure<V>) valueUnit.of(interpolated);
  }
}
