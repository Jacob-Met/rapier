//! Contacts discarded by a solver-contact hook must not retain previous impulses.

use std::sync::{
    Mutex,
    atomic::{AtomicBool, AtomicUsize, Ordering},
};

use rapier3d::pipeline::{ActiveEvents, EventHandler, PhysicsWorld};
use rapier3d::prelude::*;

#[derive(Default)]
struct ForceEvents(Mutex<Vec<Real>>);

impl EventHandler for ForceEvents {
    fn handle_collision_event(
        &self,
        _: &RigidBodySet,
        _: &ColliderSet,
        _: CollisionEvent,
        _: Option<&ContactPair>,
    ) {
    }

    fn handle_contact_force_event(
        &self,
        _: Real,
        _: &RigidBodySet,
        _: &ColliderSet,
        _: &ContactPair,
        force: Real,
    ) {
        self.0.lock().unwrap().push(force);
    }

    fn handle_soft_body_tear_event(&self, _: &SoftBodySet, _: &SoftBodyTearEvent) {}
}

struct RemoveOneContact {
    enabled: AtomicBool,
    removed: AtomicUsize,
}

impl PhysicsHooks for RemoveOneContact {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        if !self.enabled.load(Ordering::Relaxed) {
            return;
        }
        let Some(rigid) = context.rigid_mut() else {
            return;
        };
        let removed = self.removed.load(Ordering::Relaxed);
        let id = if removed == usize::MAX {
            let Some(contact) = rigid.solver_contacts.first() else {
                return;
            };
            // The high bit of a contact id is the new-contact flag.
            let id = (contact.contact_id[0] & 0x7f) as usize;
            self.removed.store(id, Ordering::Relaxed);
            id
        } else {
            removed
        };
        rigid
            .solver_contacts
            .retain(|contact| (contact.contact_id[0] & 0x7f) as usize != id);
    }
}

#[test]
fn excluded_contact_does_not_keep_its_last_impulse_or_warmstart() {
    let hooks = RemoveOneContact {
        enabled: AtomicBool::new(false),
        removed: AtomicUsize::new(usize::MAX),
    };
    let events = ForceEvents::default();
    let mut world = PhysicsWorld::new();
    let ground = world.insert_collider(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0).translation(Vector::new(0.0, -0.5, 0.0)),
        None,
    );
    let (body, box_collider) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.55, 0.0))
            .can_sleep(false),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5)
            .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
            .contact_force_event_threshold(0.0),
    );
    for _ in 0..120 {
        world.step_with_events(&hooks, &events);
    }

    let weight = world.bodies[body].mass() * 9.81;
    let dt = world.integration_parameters.dt;
    let old_pair = world
        .narrow_phase
        .contact_pair(ground, box_collider)
        .unwrap();
    let old_impulses: Vec<_> = old_pair.manifolds()[0]
        .points
        .iter()
        .map(|point| point.data.impulse)
        .collect();
    assert_eq!(old_impulses.len(), 4, "expected four contact points");
    assert!(
        (0.9..=1.1).contains(&(old_pair.total_impulse_magnitude() / dt / weight)),
        "the box must be at rest on the ground before a contact is removed"
    );

    // Remove a previously solved point, but keep it tracked by the manifold.
    // Before the fix its old impulse remains visible alongside the other three
    // points' new impulses: the reported force is 1.25 times the box's weight.
    hooks.enabled.store(true, Ordering::Relaxed);
    for _ in 0..4 {
        world.step_with_events(&hooks, &events);
    }
    let removed = hooks.removed.load(Ordering::Relaxed);
    assert_ne!(removed, usize::MAX, "the hook never saw solver contacts");
    assert!(
        old_impulses[removed] > 0.0,
        "removed point had no previous load"
    );

    let pair = world
        .narrow_phase
        .contact_pair(ground, box_collider)
        .unwrap();
    let manifold = &pair.manifolds()[0];
    assert_eq!(manifold.data.solver_contacts.len(), 3);
    assert_eq!(manifold.points.len(), 4);
    assert_eq!(manifold.points[removed].data.impulse, 0.0);
    assert_eq!(manifold.points[removed].data.warmstart_impulse, 0.0);

    let actual_force = pair.total_impulse_magnitude() / dt;
    let event_force = *events.0.lock().unwrap().last().unwrap();
    assert!(
        (0.9..=1.1).contains(&(actual_force / weight)),
        "reported {actual_force} N instead of {weight} N"
    );
    assert!(
        (0.9..=1.1).contains(&(event_force / weight)),
        "event reported {event_force} N instead of {weight} N"
    );
}
