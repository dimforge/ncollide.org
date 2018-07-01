# Collision detection pipeline

While detecting proximities and computing contacts between only two objects
might be useful to study their immediate interactions, we often work with
complex scenes involving thousands of objects. Iterating through each pair of
object and testing each of them for intersection is a $\mathcal{O}(n^2)$
process where $n$ is the number of objects. This is obviously not practicable
in real-time.


To overcome this issue, **ncollide** implements a collision detection pipeline
decomposed into two stages: the [broad phase](#broad-phase) and the [narrow
phase](#narrow-phase). The broad phase has a global knowledge of the position
of every object simultaneously so it can use spacial partitioning with
conservative interference detection algorithms to find all the potential
collision pairs very efficiently: $\mathcal{O}(n \log(n))$ time in average or
virtually $\mathcal{O}(1)$ if time coherence is high. Then the narrow phase
iterates on those pairs individually and performs the exact geometric query.
Note that some objects paired by the broad phase may not actually be in contact
(_false positives_) as it only performs approximate tests. On the other hand, a
broad phase is guaranteed not to produce any _false negative_: two interfering
objects are always guaranteed to be paired and reported to the narrow phase.


# Broad phase

A broad phase on **ncollide** must implement the `BroadPhase` trait that
ensures that it supports common geometric queries and that objects can be
added, removed, and updated.

| Method                 | Description                                     |
|--                      | --                                              |
| `.create_proxy(bv, data)`                    | Adds a new object with the bounding volume `bv`, and the _associated data_ `data` to the narrow phase. Returns an handle identifying the object inside of the broad-phase. |
| `.remove(handles, callback)`                           | Removes from the broad-phase the objects identified by the elements of the slice `handles`. The `callback` is executed on all proximity pairs that involved the removed object at the last update. |
| `.deferred_set_bounding_volume(handle, bv)`          | Informs the broad phase algorithm that the bounding volume of the object identified by `handle` has to be replaced by `bv` at the next call to `.update(...)`. |
| `.deferred_recompute_all_proximities_with(handle)` | Forces the broad phase to recompute all collision pairs involving the object identified by `handle` at the next update. |
| `.deferred_recompute_all_proximities()` | Forces the broad phase to recompute all collision pairs at the next update. |
| `.update(filter, callback)`                       | Updates this broad phase algorithm, actually performing pending object additions and removals. `filter` is a predicate that indicates if a new potential collision pair is valid. If it returns `true` for a given pair, `callback` will be called. |
| `.interferences_with_bounding_volume(bv, result)` | Fills `result` with references to each object which bounding volume intersects the bounding volume `bv`. |
| `.interferences_with_ray(ray, result)`            | Fills `result` with references to each object which bounding volume intersects `ray`. |
| `.interferences_with_point(point, result)`        | Fills `result` with references to each object which bounding volume contains `point`. |

Let us clarify what _associated data_ means here. A broad phase is guaranteed
to associate some pieces of data to each object. Those are completely
user-defined (e.g. they can even be as general as `Box<Any>`) and are passed as
argument to the user-defined callbacks when the `update` method is called.
Therefore, feel free to store in there anything that may be useful to identify
the object on your side.

Note that methods with names prefixed by `deferred_` have no effect until the
next call to `.update(...)`. This allows the broad phase to perform, e.g., one global
update even if several objects are moved individually. This update relies on a
collision filter and a callback. First, the filter should always have results
constant wrt. time. _Constant_ means that if at some time the filter returns
`true` (resp. `false`) for some object pair $(\mathcal{A}, \mathcal{B})$, it is
expected to always return `true` (resp. `false`) for $(\mathcal{A},
\mathcal{B})$ at any time in the future as well. If the filter changes at some
point (hence breaking this constancy), the method
`.deferred_recompute_all_proximities()` must be called in order to inform the
broad phase that the (new) filter should be re-executed on all potential
collision pairs already detected, and act accordingly. The second closure
`callback` passed at update-time is the bridge between the broad phase and the
narrow phase: it is called on each potential collision pair that has not
been filtered out.

Finally, a broad phase algorithm being inherently incremental, the `callback`
will usually be called only once on each new potential contact pair created or
removed as a consequence of filter change or objects being moved. Pairs
unaffected by recent changes will **not necessarily** be re-reported.

### The DBVT broad phase

The `DBVTBroadPhase` is based on a
[DBVT](../bounding_volumes/#the-bounding-volume-tree) to detect interferences
with an average $\mathcal{O}(n \log(n))$ time complexity, or even
$\mathcal{O}(1)$ if time coherence is high. Note that, instead of the exact
bounding volumes (red), the `DBVTBroadPhase` stores their loosened version
(black):

<center>
![dbvt](../img/AABB_tree_DBVT.svg)
</center>

Storing the loosened bounding volumes instead of the exact ones is a
significant optimization for scenes where the broad phase has to track contact
pairs involving slow-moving objects: the DBVT is modified by the broad phase
only when the displacement of an object is large enough to make its exact
bounding volume move out of the loosened version stored on the tree. That way
objects moving at high frequency but low amplitude will almost never trigger an
update, at the cost of slightly less tight bounding volumes for interference
detection.

Creating an empty `DBVTBroadPhase` is simple using the `::new(margin)` constructor, where `margin` controls the amount of loosening:

```rust
let mut dbvt = DBVTBroadPhase::new(0.02);
```

The following example creates four balls, adds them to a `DBVTBroadPhase`,
updates the broad phase, and removes some of them.

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#dbvt_broad_phase_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#dbvt_broad_phase_3D">3D example</a></li>
  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/build/ncollide3d/examples/dbvt_broad_phase3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/build/ncollide2d/examples/dbvt_broad_phase2d.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="dbvt_broad_phase_2D" class="tab-pane in active">
```rust
/*
 * Create the objects.
 */
let poss = [
    Isometry2::new(Vector2::new(0.0, 0.0), na::zero()),
    Isometry2::new(Vector2::new(0.0, 0.5), na::zero()),
    Isometry2::new(Vector2::new(0.5, 0.0), na::zero()),
    Isometry2::new(Vector2::new(0.5, 0.5), na::zero()),
];

// We will use the same shape for the four objects.
let ball = Ball::new(0.5);

/*
 * Create the broad phase.
 */
let mut bf = DBVTBroadPhase::new(0.2);

// First parameter: the object bounding box.
// Second parameter:  some data (here, the id that identify each object).
let proxy1 = bf.create_proxy(bounding_volume::aabb(&ball, &poss[0]), 0);
let proxy2 = bf.create_proxy(bounding_volume::aabb(&ball, &poss[1]), 1);
let _ = bf.create_proxy(bounding_volume::aabb(&ball, &poss[2]), 2);
let _ = bf.create_proxy(bounding_volume::aabb(&ball, &poss[3]), 3);

// Update the broad phase.
// The collision filter (first closure) prevents self-collision.
bf.update(&mut |a, b| *a != *b, &mut |_, _, _| {});

assert!(bf.num_interferences() == 6);

// Remove two objects.
bf.remove(&[proxy1, proxy2], &mut |_, _| {});

// Update the broad phase.
// The collision filter (first closure) prevents self-collision.
bf.update(&mut |a, b| *a != *b, &mut |_, _, _| {});

assert!(bf.num_interferences() == 1)
```
  </div>
  <div id="dbvt_broad_phase_3D" class="tab-pane">
```rust
/*
 * Create the objects.
 */
let poss = [
    Isometry3::new(Vector3::new(0.0, 0.0, 0.0), na::zero()),
    Isometry3::new(Vector3::new(0.0, 0.5, 0.0), na::zero()),
    Isometry3::new(Vector3::new(0.5, 0.0, 0.0), na::zero()),
    Isometry3::new(Vector3::new(0.5, 0.5, 0.0), na::zero()),
];

// We will use the same shape for the four objects.
let ball = Ball::new(0.5);

/*
 * Create the broad phase.
 */
let mut bf = DBVTBroadPhase::new(0.2);

// First parameter: the object bounding box.
// Second parameter:  some data (here, the id that identify each object).
let proxy1 = bf.create_proxy(bounding_volume::aabb(&ball, &poss[0]), 0);
let proxy2 = bf.create_proxy(bounding_volume::aabb(&ball, &poss[1]), 1);
let _ = bf.create_proxy(bounding_volume::aabb(&ball, &poss[2]), 2);
let _ = bf.create_proxy(bounding_volume::aabb(&ball, &poss[3]), 3);

// Update the broad phase.
// The collision filter (first closure) prevents self-collision.
bf.update(&mut |a, b| *a != *b, &mut |_, _, _| {});

assert!(bf.num_interferences() == 6);

// Remove two objects.
bf.remove(&[proxy1, proxy2], &mut |_, _| {});

// Update the broad phase.
// The collision filter (first closure) prevents self-collision.
bf.update(&mut |a, b| *a != *b, &mut |_, _, _| {});

assert!(bf.num_interferences() == 1)
```
  </div>
</div>


# Narrow phase
When the broad phase detects pairs of objects that may potentially interact, they
can be passed to the narrow phase which will instantiate the persistent
algorithms to perform exact proximity detections or contact point computations.
The narrow phase is also responsible for notifying the user when an interaction
(proximity, contact, etc.) starts or stops. Every narrow phase must implement
the `NarrowPhase` trait.

| Method | Description |
|--      | --          |
| `.update(...)` | Updates the narrow phase actually performing contact and proximity computation. |
| `.handle_interaction(..., objs, handle1, handle2, started)` | Tells the narrow phase that the objects given by `objs[handle1]` and `objs[handle2]` start or stop interacting. |
| `.handle_removal(..., objs, handle1, handle2)` | Tells the narrow phase that the interaction between `objs[handle1]` and `objs[handle2]` stopped because one of them is being removed from the world. |
| `.contact_pairs(objs)` | Returns all the contact pairs. |
| `.proximity_pairs(objs)` | Returns all the proximity pairs. |

The `.handle_interaction(...)` method will instantiate the correct persistent
contact or proximity detection algorithm for the given pair of objects. It will
also free those that are no longer relevant, e.g., because the `started` flags
is set to `false`. It will usually not perform any actual geometric query as
this is the task of the `.update(...)` method. Both methods take two _signals_
as arguments: a `ContactSignal` and a `ProximitySignal`. Both are sets of
callbacks called whenever a contact or proximity starts or stops. The user may
retrieve a list of all events that occurred during an update. Refer to the section on [event handling](#event-handling).

The `DefaultNarrowPhase` is the default implementation of the narrow phase and
should be suitable for most applications. It handles both persistent proximity
detection and persistent contact generation.

## Persistent proximity detection
Persistent proximity detection algorithms differ from the
`query::proximity(...)` [function](../geometric_queries/#proximity) in that the
formers require a structure to be instantiated. This structure can then be
re-used over time with the same shapes but with different positions and
proximity margins. This allows the proximity detector to perform significant
optimizations if the positions change only slowly over time − this is usually
called _temporal coherence_. A persistent proximity detector must implement the
`ProximityDetector` trait:

| Method | Description |
|--      | --          |
| `.update(dispatcher, ma, a, mb, b, margin)` | Performs the proximity determination between the objects `a` and `b` respectively transformed by `ma` and `mb`. |
| `.proximity(&self)` | Returns the result of the last update. |

For a given proximity detector instance, the shapes `a` and `b` are assumed
never to change over time. Changing them may lead to unexpected results. The
`margin` argument has the same semantic as with the `query::proximity(...)`
[function](../geometric_queries/#proximity). The `dispatcher` argument is a
trait-object that is responsible for instantiating the correct proximity
algorithm for a given pair of shapes. This is useful when the proximity
determination algorithm is recursive, e.g., when [composite
shapes](../geometric_representations/#composite-shapes) are involved. This
trait-object of type `ProximityDispatcher` has only one method that may return
`None` if no algorithm is known for the given shapes:

| Method                           | Description |
|--                                | --          |
| `.get_proximity_algorithm(a, b)` | Gets the persistent proximity determination algorithm dedicated to the shapes `a` and `b`. |

The `ProximityAlgorithm` return type is just an alias for a boxed
`ProximityDetector` trait-object.

If you are not interested in the whole collision detection pipeline but only in
the persistent proximity determination algorithms, the
`DefaultProximityDispatcher` can be used alone to retrieve the correct
proximity detection algorithm, depending on your geometries. You may as well
explicitly choose one from the `narrow_phase` module.


```rust
let dispatcher = DefaultProximityDispatcher::new();
let shape1 = Ball::new(0.5f32);
let shape2 = Cuboid::new(Vector2::new(0.5f32, 1.0));

let ball_vs_cuboid_detector = dispatcher.get_proximity_algorithm(&shape1, &shape2);
```

## Persistent contact generation

Persistent contact generation follows the same logic as persistent proximity
detection, but for  multiple contact points computation. Contact generators
implement the `ContactManifoldGenerator` trait.

| Method | Description |
|--      | --          |
| `.update(...)`     | Given the two shapes and their position, determines their contact geometry without returning it. |
| `.num_contacts() ` | The number of contacts manifolds generated by the last update.  |
| `.contacts(out)`   | Pushes to `out` the contact manifolds generated by the last update. |

Just like the proximity detectors, the `.update(...)` method requires a
dispatcher which is a trait-object of type `ContactDispatcher` with one
method that may return `None` if no algorithm is known for the given shapes:

| Method                           | Description |
|--                                | --          |
| `.get_contact_algorithm(a, b)` | Returns the persistent contact computation algorithm dedicated to the shapes `a` and `b`. |

The `ContactAlgorithm` return type is just an alias for a boxed
`ContactManifoldGenerator` trait-object.

If you are not interested in the whole collision detection pipeline but only in
the persistent contact determination algorithms, the
`DefaultContactDispatcher` can be used alone to retrieve the correct collision
detection algorithm, depending on your geometries. You may as well explicitly
choose one from the `narrow_phase` module.

```rust
let dispatcher = DefaultContactDispatcher::new();
let shape1 = Ball::new(0.5f32);
let shape2 = Cuboid::new(Vector2::new(0.5f32, 1.0));

let ball_vs_cuboid_detector = dispatcher.get_contact_algorithm(&shape1, &shape2);
```

### Contact manifolds
Contact generators implemented on **ncollide** generate a set contact manifolds.
A contact manifold is a set of contact points that delimit a polygonal contact area.
Therefore if you have a cube lying on a plane, one contact manifold with two (in 2D) or
four (in 3D) contact points will be returned. Those planar contacts that cannot be assimilated
to a single point are called _conforming contacts_. Approximating the shape of such contact with more than
one point is critical for, e.g., physics simulation applications with contact
laws based exclusively on a discrete number of isolated points. The following
figure shows what would happen on this kind of physics simulation with a
conforming contact approximated by only one point:

<center>
![](../img/contact.svg)
</center>

Here, the cube is falling toward the plane. When a contact is detected, the
cube is penetrating the plane and the physics engine will try to correct this
situation by applying a force at the contact point. This makes the object
rotate unrealistically, moving the contact point to the over side.  This
perpetual alternation between two contact points due to unwanted rotations
makes the simulation unstable and unrealistic. That is why **ncollide**
provides contact determination algorithms that can generate full contact manifolds.

The `ContactManifold` structure allows the retrieval of each contact point, among which the one causing the deepest penetration:

| Method                  | Description                                                  |
|--                       | --                                                           |
| `.len()`                | The number of contacts on this contact manifold.  |
| `.contacts()`           | The contacts fund for this contact manifold. |
| `.deepest_contact_id()` | The index (on the `.contacts()` array) of the contact with largest penetration depth. |
| `.deepest_contact()`    | Reference to the contact with the largest penetration depth. |

The contact points themselves contain very precise information regarding the location
of the contacts on each solid. Indeed those contacts are described by the `TrackedContact` structure
with three fields:

| Field                   | Description                                                  |
|--                       | --                                                           |
| `contact`   | The world space contact point locations, penetration depth, and normal.  |
| `kinematic` | The local configuration of the contact (more on this afterward). |
| `id`        | A unique identifier associated with this contact. `ncollide` will automatically track contact points between updates. A contact that moved between two frames will have the same `id` as in the previous frame. |

The `kinematic` field provides an approximation of contact kinematic around those contact point.
In other words, the corresponding [`ContactKinematic`](http://ncollide.org/rustdoc/ncollide3d/query/struct.ContactKinematic.html) provides:

1. A way of updating the contact location after the touching solids moved. The result of this update is approximate but very accurate if the solids did not move too much. Use the [`.contact(...)`](http://ncollide.org/rustdoc/ncollide3d/query/struct.ContactKinematic.html#method.contact) method to retrieve new contacts given the new positions of the solids.
2. Identifiers of the features on which the contact lie (`.feature1()` and `.feature2()`). This allows you to know exactly on which vertex, edge, or face the contact points are located. Refer to the documentation of each shape for a description of the meaning of these `FeatureId`. The geometry of the corresponding vertex, edge, or face, can usually be retrieve through the [`ConvexPolyhedron`](http://ncollide.org/rustdoc/ncollide3d/shape/trait.ConvexPolyhedron.html) trait.

# Collision world
The `CollisionWorld` is the main interface between the user and its
geometrical scene. It groups:

* Geometrical shapes and their positions using [collision objects](#collision-objects).
* A broad phase set to the `DBVTBroadPhase` by default.
* A narrow phase set to the `DefaultNarrowPhase` by default.

All those are hidden between a high-level interface so that the user does not
have to manually modify and synchronize the various collision detection stages.
An empty collision world is created with the constructor
`CollisionWorld::new(margin)`. The `margin` argument is only for
optimization purpose and has the same semantic as its
[counterpart](#the-dbvt-broad-phase) for the creation of the DBVT broad-phase:
it is the amount of loosening for each bounding volume (this does not
affect the exact geometric shapes). While this value depends on your specific
application, a value of 0.02 is usually good enough if your objects have an
average size of 1 (no matter which
[units](../faq/#what-units-are-used-by-ncollide) you use).

## Collision objects
Instances of the `CollisionObject` structure are the main citizens of the
collision world. They contain all information needed to describe a shape and
its position in space:

| Method              | Description                                                  |
|--                   | --                                                           |
| `.handle()`         | The handle of this collision object on the collision world.  |
| `.proxy_handle()`   | The handle of this collision object on the broad-phase structure. |
| `.position()`         | The collision object position in space.                      |
| `.shape()`            | The geometrical shape of the collision object.               |
| `.collision_groups()` | Groups used to prevent interactions with some other objects. |
| `.query_type()`       | The kind of query this object can be involved in.            |
| `.data()`             | User-defined data associated to the collision object. This will never be modified by **ncollide**. |
| `.data_mut()`         | Mutable reference to the user-defined data associated to the collision object. |

The two methods `.collision_groups()` and `.query_type()` affect how the object will
interact with the others on the collision world and are detailed in the next
sections.

A collision object should not be created directly by the user. Instead it
is initialized internally by the collision world with the
`CollisionWorld::add(...)` method. The `CollisionObject` instance created by the world can be
retrieved using the `.collision_object(handle)` method, where `handle` is the the return
value of the collision world's `.add(...)` method. Currently, a collision object can
only be moved or removed from the collision world. Future versions of
**ncollide** will allow you to modify its shape, collision groups, and query
types as well. All those modifications will wait until the next call to
`.update()` to be actually performed efficiently:

| Field                     | Description |
|--                         | --          |
| `.set_position(handle, pos)` | Sets the position of the collision object identified by `handle`. Collision detection with other objects will occur only at the next update of the collision world. |
| `.remove(handle)`            | Removes a set of collision object identified by their handles. |

### Collision groups
Collision groups are the main way to prevent some objects from interacting with
each other. Internally, it is represented as a few bit fields such that
verifying if two objects can interact is only a matter of performing bitwise
operations.  The `CollisionGroups` structure defines three masks:

1. The **membership** mask − the list of groups this collision object is part
   of.
2. The **whitelist** mask − the list of groups this collision object can
   interact with if it is not blacklisted.
3. The **blacklist** mask − the list of groups this collision object must never
   interact with.

Note that the blacklist has precedence over the whitelist, i.e., if a group is
on both lists, interactions will be forbidden anyway. Setting up a
`CollisionGroups` structure must follow the following reasoning: each object
can be member of several group. If it encounters another object, a geometric
query will be performed by the narrow phase if and only if:

* Each object is member of **at least one** group part of the other object's whitelist.
* **None** of the object is member of any group part of the other object's
  blacklist.

A collision group is identified by a positive number on the range $[\![ 0, 29 ]\!]$.
Thus, only 30 groups are available. By default, the `CollisionGroups::new()`
constructor will initialize the masks such that the object is member of all
groups, whitelists all groups, and blacklists none of them. You may modify,
set, or copy those masks with the corresponding methods prefixed by `.modify_`,
`.set_`, and `.copy_`.

Finally, there exists one special group for self-collision. Because this is
meaningful only for deformable shapes (which are not yet explicitly supported
by **ncollide**), it is disabled by default. It may be enabled with the
`.enable_self_collision()` method. This will allow the narrow phase to perform
a geometric query involving this collision object twice.

#### Example <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/collision_groups.rs')"></div>

In the following example, we have three `CollisionGroups` A, B, and C. The
first one is such that:

* It is member of the groups 1, 3, and 6.
* It whitelists the groups 6 and 7.
* It blacklists the group 1.

The second one is such that:

* It is member of the groups 1, 3, and 7.
* It whitelists the groups 3 and 7.
* It does not blacklist anything.

The third one is such that:

* It is member of the groups 6 and 9.
* It whitelists the groups 3 and 7.
* It does not blacklist anything.

As a result, we observe that:

* A and B will **not** interact because B is part of the group 1 which is
  blacklisted by A.
* B and C will **not** interact either because, even if C whitelists the group
  3 (which B is part of), B does not whitelists neither the group 6 nor the
  group 9 (which C is part of).
* A and C **will** interact because A whitelists the group 6 (which C is part
  of), and, reciprocally, C whitelists the group 3 (which A is part of).
  In addition, C is not part of the group 1 which is blacklisted by A.

```rust
let mut a = CollisionGroups::new();
let mut b = CollisionGroups::new();
let mut c = CollisionGroups::new();

a.set_membership(&[1, 3, 6]);
a.set_whitelist(&[6, 7]);
a.set_blacklist(&[1]);

b.set_membership(&[1, 3, 7]);
b.set_whitelist(&[3, 7]);

c.set_membership(&[6, 9]);
c.set_whitelist(&[3, 7]);

assert!(!a.can_interact_with_groups(&b));
assert!(!b.can_interact_with_groups(&c));
assert!(a.can_interact_with_groups(&c));
```

### Custom collision filters
If collision groups are not flexible enough for your specific needs, you may
register arbitrary filters with the collision world's
`.register_broad_phase_pair_filter(name, filter)` method. A filter may be
removed using his name with `.unregister_broad_phase_pair_filter(name)`.
Custom filters are called only if the collision groups do not reject the
interaction. If multiple filters are registered, they are applied successively
until one returns `false`. The interaction is allowed for the tested pair iff.
they all return `true`. A collision filter must implement the
`BroadPhasePairFilter` trait:

| Method                     | Description |
|--                          | --          |
| `.is_pair_valid(co1, co2)` | Returns `true` if the two collision objects are allowed to interact. |

The same _constancy_ rule as for the [broad phase](#broad-phase) `.update(...)`
filter argument applies here. Thus if you which to modify you filter's
semantic, it has to be removed and re-added.

The following examples add to the collision world a very arbitrary rule that
filters interactions in a way that would be impossible using collision groups.
It allows interactions between collision objects with identifiers sharing the
same parity only. For this example, the collision object user-data is set to
`()`.

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#custom_collision_filter_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#custom_collision_filter_3D">3D example</a></li>
  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/build/ncollide3d/examples/custom_collision_filter3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/build/ncollide2d/examples/custom_collision_filter2d.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="custom_collision_filter_2D" class="tab-pane in active">
```rust
struct ParityFilter;

impl BroadPhasePairFilter<f32, ()> for ParityFilter {
    fn is_pair_valid(
        &self,
        b1: &CollisionObject<f32, ()>,
        b2: &CollisionObject<f32, ()>,
    ) -> bool {
        b1.handle().uid() % 2 == b2.handle().uid() % 2
    }
}
```
  </div>
  <div id="custom_collision_filter_3D" class="tab-pane">
```rust
struct ParityFilter;

impl BroadPhasePairFilter<f32, ()> for ParityFilter {
    fn is_pair_valid(
        &self,
        b1: &CollisionObject<f32, ()>,
        b2: &CollisionObject<f32, ()>,
    ) -> bool {
        b1.handle().uid() % 2 == b2.handle().uid() % 2
    }
}
```
</div>
</div>

Then, this filter should be added to the collision world with:
```rust
world.register_broad_phase_pair_filter("Parity filter", ParityFilter);
```

### Query type
The query type stored in the `.query_type` field of collision objects indicates
which kinds of geometric queries should be executed by the narrow phase on it.
Two choices are given by the `GeometricQueryType` enumeration. More queries
for, e.g., minimal distance computation, may be allowed in the future:

1. `::Contacts(prediction, ang_prediction)` − two objects with this query
   type will have their [contact points](../geometric_queries/#contact)
   computed. Contact points will be generated as long as the two objects are
   penetrating or closer than the sum of both `prediction` values. `ang_prediction`
   will allow the generation of contacts between two features (vertices, edges, or faces) of solids that should be in contact if the solid were rotated by this amount. A value of `0.09` radians (around 5 degrees) provide good results. Setting prediction values that are too large may deteriorate performance since this may cause the generation of a large number of contact points. Small prediction values may deteriorate the quality of contact tracking.
2. `::Proximity(margin)` − if at least one object has this query type, then
   only [proximity detection](../geometric_queries/#proximity) will be
   performed. Shapes separated by a distance larger than the sum of their
   `margin` will be considered disjoint.

If the two shapes request different query types, only the simplest is
performed. For example, one shape having a `::Contacts(prediction1, ang_prediction2)` query type
interacting with a shape with a `::Proximity(margin2)` query type will result
in a proximity query. In other words, the first `::Contacts(...)` query type is
implicitly downgraded to `::Proximity(...)`.

## World-scale geometric queries
Because the collision world groups collision objects with efficient
acceleration data structures, it is natural to give the user the ability to
apply [single-shape](./geometric_queries/#single-shape-queries) geometric
queries to all the objects simultaneously:

| Method                 | Description |
|--                      | --          |
| `.interferences_with_ray(ray, groups)` | Returns an iterator through all objects able to interact with `groups` and intersecting `ray`. |
| `.interferences_with_point(point, groups)` | Returns an iterator through all objects able to interact with `groups` and containing `point`. |
| `.interferences_with_aabb(aabb, groups)` | Returns an iterator through all objects able to interact with `groups` and with an AABB that intersects `aabb`.  |

Not all collision objects have to be affected by the geometric query: only
those with a collision groups that can interact with `groups` will actually
execute the query. The results of pairwise geometric queries performed by the
narrow phase can be retrieved as well through the collision world:

| Method                | Description |
|--                     | --          |
| `.contact_pair(handle1, handle2)`  | Gets the contact computation algorithm allocated for the given pair of objects. Returns `None` if the specified objects are too far apart to be handled by the narrow phase. |
| `.contact_pairs()`    | Gets an iterator through the contact pairs created by the narrow phase.   |
| `.proximity_pairs()`  | Gets an iterator through the proximity pairs created by the narrow phase. |
| `.contact_manifolds()` | Gets an iterator through the contact manifolds computed by the narrow phase. |

## Event handling
It is often useful to react when two objects start/stop being in contact or
when their proximity status change. For example, when two object start
colliding we might want to play a sound, display a message, apply some AI
logic, etc. Those status changes can be detected by the user by retrieving
iterators yielding events from the collision world. The [next section](#example_1)
provides a detailed example of event handling.

### Proximity event
Proximity events are triggered when two collision objects subject to proximity
queries transition between two different proximity statuses. There are three
possible proximity statuses:

1. `Proximity::Intersecting` indicates two collision objects are touching/penetrating.
2. `Proximity::WithinMargin` indicates two collision objects are not touching, but are separated by a distance smaller than the sum of tolerances specified by the `GeometricQueryType::Proximity(...)` given to each collision object at initialization.
3. `Proximity::Disjoint` indicates two collision objects are no longer touching, and are too far apart to be within margins.

An iterator through all proximity events may be retrieved using the `.proximity_events()`
method of the `CollisionWorld`. This yields a `ProximityEvent` structure:

| Field                | Description |
|--                     | --          |
| `collider1` | The handle of the first collision object involved in the proximity. |
| `collider2` | The handle of the second collision object involved in the proximity. |
| `prev_status` | The previous proximity status of the collision objects. |
| `new_status`  | The current proximity status of the collision objects. |

Only transitions are reported, so `prev_status != new_status` is guaranteed. On the other
hand, keep in mind that collision objects are not required to have smooth
motions. Thus, the transition from, e.g., `Proximity::Intersecting` to
`Proximity::Disjoint` is possible even if a smooth motion would have
necessarily triggered transitions from `::Intersecting` to `::WithinMargin` and
then from `::WithMargin` to `::Disjoint` instead.


### Contact event
Contact events are triggered when two collision objects subject to contact
queries transition between having zero contact point and at least one.
Transitioning between one to more than one contact points is not reported.

An iterator through all contact events may be retrieved by the `.contact_events()`
method of the `CollisionWorld`. The `ContactEvent` enum has two variants:

1. `ContactEvent::Started(handle1, handle2)` to indicate the transition between
0 to at least one contact point.
2. `ContactEvent::Stopped(handle1, handle2)` to indicate the transition between
at least one contact point to 0.

In case of starting contacts, it is possible to retrieve the contacts by obtaining
a reference to the contact algorithm with the world's `.contact_pair(handle1, handle2)`
method. The resulting contact algorithm (a `ContactManifoldGenerator` trait-object) allows the retrieval of the computed
contact manifolds. Refer to the [`ContactManifoldGenerator`](http://ncollide.org/rustdoc/ncollide3d/narrow_phase/trait.ContactManifoldGenerator.html)
trait.

## Example <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/ncollide_testbed3d/examples/bouncing_ball3d.rs')"></div><div class="sp"></div><div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/ncollide_testbed2d/examples/bouncing_ball2d.rs')"></div>

This detailed example simulates the trajectory of a 2D ball trapped into a
square-shaped room. The room itself is split into four areas of different
colours. When the ball hits a wall, it bounce against it following a simple
reflection along the contact normal. The coloured areas act as sensors: a
message is printed whenever the ball enters and leaves any of them. Bouncing will
occur after a contact event while entering and leaving coloured areas will be
detected through proximity events. The proposed implementation is not the most
efficient one but has the benefit to illustrate several features of the
collision detection pipeline. A 3D version of this example is almost identical
and may be downloaded using the button above.

<center>
![bouncing balls](../img/bouncing_ball.gif)
</center>

We first define a `CollisionObjectData` structure that will be attached to our
collision objects. When a proximity or contact event occurs, we need to be able
to:

1. Identify each wall. We give each area a name as a string. This name will be
   directly used by the message printer as the ball enters or leaves the area.
   (Note that instead of a string we could have simply used the collision
   object `.handle()` method to identify the wall. Though a string is more
   user-friendly here.)
2. Be able to modify the ball velocity. Here, we enclose the velocity into a `Cell`
   for simplicity to benefit from interior mutability. The ball is the only collision
   object with a velocity different than `None`.

```rust
#[derive(Clone)]
struct CollisionObjectData {
    pub name:     &'static str,
    pub velocity: Option<Cell<Vector2<f32>>>
}

impl CollisionObjectData {
    pub fn new(name: &'static str, velocity: Option<Vector2<f32>>) -> CollisionObjectData {
        let init_velocity;
        if let Some(velocity) = velocity {
            init_velocity = Some(Cell::new(velocity))
        }
        else {
            init_velocity = None
        }

        CollisionObjectData {
            name:     name,
            velocity: init_velocity
        }
    }
}
```

Our proximity handler will by the `handle_proximity_event(...)` function.
When a proximity event occurs, we first find the area's name by taking
the one from the collision object without velocity (because the only one
with a velocity is the ball). Then the new proximity
status is tested to display the corresponding message. The
`Proximity::WithinMargin` case will never occur because our collision objects
will be initialized with a zero proximity prediction margin, i.e., with `GeometricQueryType(0.0)`.

```rust
fn handle_proximity_event(
    world: &CollisionWorld<f32, CollisionObjectData>,
    event: &ProximityEvent,
) {
    // The collision object with a None velocity is the coloured area.
    let area_name;
    let co1 = world.collision_object(event.collider1).unwrap();
    let co2 = world.collision_object(event.collider2).unwrap();

    if co1.data().velocity.is_none() {
        area_name = co1.data().name;
    } else {
        area_name = co2.data().name;
    }

    if event.new_status == Proximity::Intersecting {
        println!("The ball enters the {} area.", area_name);
    } else if event.new_status == Proximity::Disjoint {
        println!("The ball leaves the {} area.", area_name);
    }
}
```

Contacts will be handled by the `handle_contact_event(...)` function.
When the contact event is `ContactEvent::Started(...)`,
we already have the guarantee that at least one contact is available.
We first collect the contacts with the wall then the
velocity of the ball can be modified by flipping its component along the first
contact normal.

```rust
fn handle_contact_event(world: &CollisionWorld<f32, CollisionObjectData>, event: &ContactEvent) {
    if let &ContactEvent::Started(collider1, collider2) = event {
        // NOTE: real-life applications would avoid this systematic allocation.
        let pair = world.contact_pair(collider1, collider2).unwrap();
        let mut collector = Vec::new();
        pair.contacts(&mut collector);

        let co1 = world.collision_object(collider1).unwrap();
        let co2 = world.collision_object(collider2).unwrap();

        // The ball is the one with a non-None velocity.
        if let Some(ref vel) = co1.data().velocity {
            let normal = collector[0].deepest_contact().unwrap().contact.normal;
            vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * *normal);
        }
        if let Some(ref vel) = co2.data().velocity {
            let normal = -collector[0].deepest_contact().unwrap().contact.normal;
            vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * *normal);
        }
    }
}
```

Finally, we have to setup the collision world:

1. Initialize the shapes. All the coloured areas can share the same cuboid.
2. Initialize the collision groups. We do not want any proximity event to be
   reported between two coloured areas or between a wall and coloured area.  We
   do not want to detect contacts between two planes either. This filtering is
   easily done using [collision groups](#collision-groups). The ball is
   member of a group different from all the other objects. Then, the other
   objects whitelist the ball only so that they will not interact with each
   other.
3. Run the simulation loop. We start by polling events using `world.proximity_events()`
   and `world.contact_events()` and passing the to our handlers. Then we update the
   ball's position.

```rust
/*
 * Setup initial object properties.
 */
// Plane shapes.
let plane_left   = ShapeHandle::new(Plane::new(Vector2::x_axis()));
let plane_bottom = ShapeHandle::new(Plane::new(Vector2::y_axis()));
let plane_right  = ShapeHandle::new(Plane::new(-Vector2::x_axis()));
let plane_top    = ShapeHandle::new(Plane::new(-Vector2::y_axis()));

// Shared cuboid for the rectangular areas.
let rect = ShapeHandle::new(Cuboid::new(Vector2::new(4.9f32, 4.9)));

// Ball shape.
let ball = ShapeHandle::new(Ball::new(0.5f32));

// Positions of the planes.
let planes_pos = [
    Isometry2::new(Vector2::new(-10.0, 0.0), na::zero()),
    Isometry2::new(Vector2::new(0.0, -10.0), na::zero()),
    Isometry2::new(Vector2::new(10.0, 0.0),  na::zero()),
    Isometry2::new(Vector2::new(0.0,  10.0), na::zero())
];

// Position of the rectangles.
let rects_pos = [
    Isometry2::new(Vector2::new(-5.0, 5.0),  na::zero()),
    Isometry2::new(Vector2::new(5.0, 5.0),   na::zero()),
    Isometry2::new(Vector2::new(5.0, -5.0),  na::zero()),
    Isometry2::new(Vector2::new(-5.0, -5.0), na::zero())
];

// Position of the ball.
let ball_pos = Isometry2::new(Vector2::new(5.0, 5.0), na::zero());

// The ball is part of group 1 and can interact with everything.
let mut ball_groups = CollisionGroups::new();
ball_groups.set_membership(&[1]);

// All the other objects are part of the group 2 and interact only with the ball (but not with
// each other).
let mut others_groups = CollisionGroups::new();
others_groups.set_membership(&[2]);
others_groups.set_whitelist(&[1]);

let plane_data       = CollisionObjectData::new("ground", None);
let rect_data_purple = CollisionObjectData::new("purple", None);
let rect_data_blue   = CollisionObjectData::new("blue", None);
let rect_data_green  = CollisionObjectData::new("green", None);
let rect_data_yellow = CollisionObjectData::new("yellow", None);
let ball_data        = CollisionObjectData::new("ball", Some(Vector2::new(10.0, 5.0)));

/*
 * Setup the world.
 */
// Collision world 0.02 optimization margin and small object identifiers.
let mut world = CollisionWorld::new(0.02, true);

// Add the planes to the world.
let contacts_query  = GeometricQueryType::Contacts(0.0, 0.0);
let proximity_query = GeometricQueryType::Proximity(0.0);

world.add(planes_pos[0], plane_left,   others_groups, contacts_query, plane_data.clone());
world.add(planes_pos[1], plane_bottom, others_groups, contacts_query, plane_data.clone());
world.add(planes_pos[2], plane_right,  others_groups, contacts_query, plane_data.clone());
world.add(planes_pos[3], plane_top,    others_groups, contacts_query, plane_data.clone());

// Add the colored rectangles to the world.
world.add(rects_pos[0], rect.clone(), others_groups, proximity_query, rect_data_purple);
world.add(rects_pos[1], rect.clone(), others_groups, proximity_query, rect_data_blue);
world.add(rects_pos[2], rect.clone(), others_groups, proximity_query, rect_data_green);
world.add(rects_pos[3], rect.clone(), others_groups, proximity_query, rect_data_yellow);

// Add the ball to the world.
let ball_handle = world.add(ball_pos, ball, ball_groups, contacts_query, ball_data);

// Register our handlers.
world.register_proximity_handler("ProximityMessage", ProximityMessage);
world.register_contact_handler("VelocityBouncer", VelocityBouncer);

/*
 * Run indefinitely.
 */
let timestep = 0.016;

loop {
    // Poll and handle events.
    for event in world.proximity_events() {
        handle_proximity_event(&world, event)
    }

    for event in world.contact_events() {
        handle_contact_event(&world, event)
    }

    // Integrate velocities and positions.
    let ball_pos;
    {
        // Integrate the velocities.
        let ball_object   = world.collision_object(ball_handle).unwrap();
        let ball_velocity = ball_object.data.velocity.as_ref().unwrap();

        // Integrate the positions.
        ball_pos = ball_object.position.append_translation(&(timestep * ball_velocity.get()));
    }

    // Submit the position update to the world.
    world.set_position(ball_handle, ball_pos);
    world.update();
}
```

The following should be printed on the console:

```markdown
The ball enters the blue area.
The ball enters the purple area.
The ball leaves the blue area.
The ball enters the yellow area.
The ball leaves the purple area.
The ball enters the green area.
The ball leaves the yellow area.
The ball enters the yellow area.
The ball leaves the green area.
The ball enters the purple area.
The ball leaves the yellow area.
The ball enters the blue area.
The ball leaves the purple area.
...
```
