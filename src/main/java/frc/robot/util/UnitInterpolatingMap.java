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

  @SafeVarargs
  public static <K extends Unit, V extends Unit> UnitInterpolatingMap<K, V> ofEntries(
      K keyUnit, V valueUnit, Measure<K>[] keys, Measure<V>... values) {
    if (keys.length != values.length) {
      throw new IllegalArgumentException("keys and values must have the same length");
    }

    var map = new UnitInterpolatingMap<>(keyUnit, valueUnit);
    for (int i = 0; i < keys.length; i++) {
      map.put(keys[i], values[i]);
    }
    return map;
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
