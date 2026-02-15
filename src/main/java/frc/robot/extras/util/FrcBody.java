package frc.robot.extras.util;

import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.math.forces.Velocity2d;
import frc.robot.extras.math.mathutils.GeomUtil;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.PhysicsBody;
import org.dyn4j.dynamics.contact.ContactConstraint;

/** A Dyn4j Body with additional methods for serialization and conversion to WPILib types. */
public class FrcBody extends Body {

  // Track collisions manually
  private transient List<PhysicsBody> collidingBodies = new ArrayList<>();
  private transient boolean isCollidingCache = false;
  private transient int collisionCountCache = 0;

  public record FrcBodySnapshot(
      Pose2d pose,
      Mass mass,
      MomentOfInertia momentOfInertia,
      Velocity2d velocity,
      AngularVelocity angularVelocity,
      double linearDamping,
      double angularDamping,
      double gravityScale,
      boolean isBullet,
      double atRestTime,
      Translation2d forces,
      Torque torque,
      Translation2d accumulatedForce,
      Torque accumulatedTorque,
      boolean isColliding,
      int collisionCount)
      implements StructSerializable {
    public static final Struct<FrcBodySnapshot> struct =
        StructGenerator.genRecord(FrcBodySnapshot.class);
  }

  public FrcBodySnapshot snapshot() {
    return new FrcBodySnapshot(
        GeomUtil.toWpilibPose2d(getTransform()),
        GeomUtil.toWpilibUnit(getMass()).getFirst(),
        GeomUtil.toWpilibUnit(getMass()).getSecond(),
        new Velocity2d(getLinearVelocity().x, getLinearVelocity().y),
        RadiansPerSecond.of(-getAngularVelocity()),
        getLinearDamping(),
        getAngularDamping(),
        getGravityScale(),
        isBullet(),
        this.atRestTime,
        new Translation2d(getForce().x, getForce().y),
        NewtonMeters.of(getTorque()),
        new Translation2d(getAccumulatedForce().x, getAccumulatedForce().y),
        NewtonMeters.of(getAccumulatedTorque()),
        isColliding(),
        getCollisionCount());
  }

  public boolean isColliding() {
    return collidingBodies != null && !collidingBodies.isEmpty();
  }

  public int getCollisionCount() {
    return collidingBodies != null ? collidingBodies.size() : 0;
  }

  public List<PhysicsBody> getCollidingBodies() {
    if (collidingBodies == null) {
      collidingBodies = new ArrayList<>();
    }
    return collidingBodies;
  }

  /** Adds a body to the collision list (used by the arena). */
  public void addCollidingBody(PhysicsBody body) {
    if (collidingBodies == null) {
      collidingBodies = new ArrayList<>();
    }
    if (!collidingBodies.contains(body)) {
      collidingBodies.add(body);
    }
    isCollidingCache = true;
    collisionCountCache = collidingBodies.size();
  }

  public void updateCollisions(List<ContactConstraint> contacts) {
    clearCollisions();
    if (contacts != null) {
      for (ContactConstraint constraint : contacts) {
        PhysicsBody body1 = constraint.getBody1();
        PhysicsBody body2 = constraint.getBody2();
        if (body1 == this && body2 != null && body2 != this) {
          addCollidingBody(body2);
        } else if (body2 == this && body1 != null && body1 != this) {
          addCollidingBody(body1);
        }
      }
    }
  }

  public void clearCollisions() {
    if (collidingBodies != null) {
      collidingBodies.clear();
    }
    isCollidingCache = false;
    collisionCountCache = 0;
  }
}
