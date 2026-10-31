# Limits

Why the magnitudes `oxijolt::limits` accepts have the values they have: for each bound, the Jolt
arithmetic it keeps finite. [coverage.md](coverage.md) lists every input with its rule and boundary
test, the paths only tests cover, and what no bound excludes. Line numbers refer to the vendored
Jolt 5.6 sources (`crates/oxijolt-sys/vendor/JoltPhysics/Jolt/Physics/`).

The derivations hold with every input at its bound and a time step `dt <= 1` s
(`PhysicsWorld::MAX_DELTA_TIME`); every collision step lies between `PhysicsWorld::MIN_DELTA_TIME`
and that. Numbers are rounded.

## Frame and extent

`MAX_POSITION` follows Jolt's "Big Worlds" documentation (`Docs/Architecture.md`): single-precision
simulation is accurate within roughly 5 km of the origin, and double precision handles worlds of
thousands of km; at 10 000 km Jolt's `f32` broad phase still has a resolution of about 1 m. It is
not a Jolt assertion threshold.

`MAX_SHAPE_EXTENT` follows Jolt's "Conventions and Limits" documentation, which recommends static
objects of 0.1 to 2000 m; the bound applies on each side of the centre of mass. It bounds a shape's
inertia to at most `6 · mass · MAX_SHAPE_EXTENT²`. A compound, also one a `MutableCompound`
publishes, is checked after Jolt moved its centre of mass to the children's mass-weighted centre,
so children whose positions are each within the bound can still leave it.

A plane (`Shape::new_plane`) is bounded through its local bounds like any other shape. Jolt puts
them around the square of `2 · half_extent` metres centred on `-constant · normal` and the same
square moved `half_extent` behind the plane (`PlaneShape.cpp:46-81`). For a normal along an axis
they reach `max(|constant|, |constant + half_extent|)` along it, so with normal +Y a half extent of
2000 m fits a plane at y = 1 (`constant = -1`) but not one at y = -1; a tilted normal spreads the
square over two or three axes. The constructor checks `|constant|` and `half_extent` against
`MAX_SHAPE_EXTENT` first and Jolt's computed bounds after.

## Expanded compounds

Jolt walks a compound's children without remembering the shapes it has already seen.
`CompoundShape::GetSubShapeIDBitsRecursive` and `CompoundShape::GetMassProperties` call every
child (`Collision/Shape/CompoundShape.cpp:68-90, 117-123`). The static compound constructor calls
both (`StaticCompoundShape.cpp:204, 349`), so does the mutable one (`MutableCompoundShape.cpp:48,
78`), and a body reads the mass properties when it is created. A shape that several children share
is walked once per use. A compound that holds the level below twice at each of `d` levels therefore
costs `2^(d+1) - 1` calls per walk. At `d = 31`, which still fits the 32-bit sub-shape id rule,
that is about 4.3e9 calls, minutes of work from one safe call.

The safe API counts that expanded tree: the compound itself, plus the expanded tree of each child at
every use, with decorators and leaves counting one each. It refuses a compound above
`MAX_EXPANDED_SUB_SHAPES` = 2^20 = 1 048 576 with `ShapeError::TooManySubShapes`. The count is kept
per distinct shape, so a refusal costs one visit per distinct shape. `MutableCompound` checks it
before every edit.

Measured on the development machine (Windows, debug test build over the release Jolt library) with
such a shared graph, the cost grows linearly at about 35-40 ns per expanded shape. At 2^20 - 1
shapes, building the compound took 39 ms, creating a dynamic body with it 59 ms, and a step with that
body awake and touching nothing 10 ms; a cube resting on a static compound of 2^20 overlapping
leaves cost about 50 ms per step. A flat compound of 2^20 distinct children walks as many shapes, so the bound
counts sharing at what it costs, not as an error, and still leaves room for flat compounds far
larger than a game chunk.

## Convex hulls

`Shape::new_convex_hull` replays the start of Jolt's hull builder (`ConvexHullBuilder::Initialize`,
`Geometry/ConvexHullBuilder.cpp:300-470`) in `f64` before Jolt sees the points: the first point
farthest from the origin, the first point farthest from it, the point that makes the largest
triangle with both, and the point farthest from that triangle's plane. Write `L` for the distance
of the first two points, `w` for the third point's distance from their line (no point lies farther
from it), `t` for the farthest point's distance from the triangle's plane, `c` for Jolt's coplanar
distance `3 · FLT_EPSILON · (max |x| + max |y| + max |z|)` (`DetermineCoplanarDistance`), and `T` for
`max(1e-3 m, c)`, the tolerance the builder uses (`ConvexHullShapeSettings::mHullTolerance` is 1 mm).
The constructor refuses:

- fewer than 4 points: Jolt accepts 3, but a triangle has no volume;
- `|(p1 - p) × (p2 - p)|² < 1e-12` for every third point `p`, Jolt's `cMinTriangleAreaSq`, as
  `Degenerate`;
- `w · T < 0.25 · L · c` as `Degenerate`: rounding a position by about `c` tilts a face built
  across the width by `c / w` and moves it by `L · c / w` at the far end, which must stay well inside
  the tolerance;
- `t < 200 · c` as `Coplanar`. Jolt itself builds a flat hull of two faces up to `6 · c`
  (`cCoplanarSlopFactor`), which gives a dynamic body zero mass, and asserts on slabs a little
  thicker.

The two last bounds are measured, not derived. In an asserts build, 37 440 near-line and
near-plane clouds (lengths 0.1 to 1900 m, 5 to 200 points, up to 900 m from the origin) asserted
396 times. The 395 needles among them asserted at `ConvexHullBuilder.cpp:641`
(`edges.size() >= 3`), all with `w · T / (L · c)` at most 0.052, so the needle bound keeps a factor
of about 5; the slab bound refuses the other one. Slabs asserted up to `t / c = 10.9`, and a
seeded run found one cloud of 279 points on a sphere of radius 0.047 m about 1 km from the origin
that asserted at `t / c = 60.3` (`ConvexHullBuilder.cpp:779`, `IsFacing`); the slab bound keeps a
factor of about 3 over it. With both bounds in place, 40 000 random clouds of aspect ratios
1e-7 to 1, sizes 1 mm to 1900 m and offsets up to 1 km, 30 000 compact clouds 5 to 5000 coplanar
distances wide placed up to 1900 m from the origin, and seeded stress runs with dropped and
queried bodies, among them well-spread clouds with a nearly collinear spike, a nearly flat cap
above one face or near copies of their extreme points in shuffled order, built and stepped
without an assertion.

Near the origin `200 · c` is a fraction of a millimetre (a 2 m plank must be about 0.14 mm thick);
1.8 km out it is about 0.21 m. Both bounds grow with the distance of the points from the shape
origin: centre the points on the origin for the thinnest hulls. Near a bound Jolt's `f32`
arithmetic can differ from the `f64` replay; Jolt then refuses with its own message.

### Clouds the hull builder asserts on

Both bounds measure the builder's initial simplex, while the assertions sit in later steps (the
horizon of a new point, the facing test of a later face). With the `asserts` feature Jolt's builder
therefore still aborts on some clouds that pass them, all with many nearly coplanar faces:

- faces sampled densely and moved a few coplanar distances off their planes, as scanned or
  decimated geometry is: about one in 1000 such boxes with `t / c` between 200 and 1000 asserted,
  none of 92 000 above 2000, and 1 of 2953 sized like the cones below, at 210;
- dense flat cones and domes, a rim of hundreds of points a few coplanar distances off its plane
  under an apex. Sized across 200 to 6000 coplanar distances, 40 of 2958 such caps and domes
  (1.4 %) asserted, at `t / c` from about 350 to 10 000, and 64 of 2997 far cones moved to the
  origin by the caller (2.1 %), from about 280 to 7700.

They abort at `ConvexHullBuilder.cpp:1220`
(`e->mNeighbourEdge->mFace != other_edge->mNeighbourEdge->mFace`), `:779` (`IsFacing`) or `:858`,
from `t / c` of about 210. No slab bound separates them from good clouds: one above `1e5 · c` would
refuse a 2 m plank thinner than 7 cm near the origin. This is a limit of the `asserts` feature.

Without it Jolt refuses most of these clouds itself with "Hull building failed"
(`ShapeError::Rejected`) and builds a hull from the rest: of the 105 clouds that asserted in the
sizing run above, 92 were refused and 13 built. `hull_shapes.rs` checks six refused and two
built ones. The seeded stress, which runs with asserts too, draws sparse clouds only.
Recentring a far cloud does not move it out of the second group: the cloud keeps the rounding of
its far coordinates, about a twentieth of its coplanar distance there, and at the origin that
rounding is several coplanar distances of the moved cloud.

Jolt also refuses, in every build, about one cloud in eight of densely sampled boxes whose faces are
noisy by up to 10 coplanar distances, with "Hull building failed": thin out scanned or decimated
clouds, or snap their faces.

Jolt keeps at most 256 vertices of a hull (`cMaxPointsInHull`) and drops the points inside it. It
reduces the convex radius until twice the radius fits the hull's thinnest direction and its
sharpest edges (`ConvexHullShape.cpp:269-345`).

### Thin dynamic hulls on a floor

The cause is the impact, not the hull. Flat cones and domes 0.4 to 7 cm thick and 1 to 14 m
wide, set down 1 cm above a box floor base down or dome down, come to rest within 0.5 mm of the
floor and fall asleep; convex radius 0 and 0.05 behave alike. Dropped tilted from 3 m, such a
hull lands on its rim, tips over and slaps down: its far edge moves at the radius times the
angular velocity (about 5 m/s for a 4 m dome), while Jolt's discrete step reacts to an approach
only within its speculative contact distance (`mSpeculativeContactDistance`, 2 cm per step by default) and
`MotionQuality::LinearCast` sweeps the centre of mass's translation, not the rotation.

Measured with twelve random tilts per hull, 60 Hz, no convex radius (`tests/thin_hulls.rs` keeps
the 4 m wide, 1 cm thick dome as its gate):

- Box or plane floor: the slapping edge of the 1 cm dome sinks up to 24 cm, of the 1 cm flat
  cone up to 13 cm. Flat cones then recover to the penetration slop (2 cm). Six of twelve domes
  stay wedged about 11 cm deep after 6 s, rocking with a vertical velocity near 0.5 m/s: the
  sinking seen first.
- Flat mesh floor (a surface with nothing behind it): the edge passes through and the hull
  falls. 7 to 12 of twelve cones and domes 0.4 to 7 cm thick fell through, one of twelve 20 cm
  thick 4 m wide flat cones, and 3 and 6 of twelve 50 cm thick 14 m wide cones and domes; 20 cm
  thick 1 m wide ones did not.
- With `MotionQuality::LinearCast` none of the twelve 1 cm, 4 m domes stayed wedged in the box
  floor (one of twelve 3 cm, 14 m domes did), and 6 of twelve instead of 11 fell through the
  mesh. Four steps of 1/240 s per frame left none wedged and let 7 of twelve through the mesh.

Wide thin dynamic shapes therefore belong on box or convex floors rather than meshes;
`LinearCast` and shorter steps reduce, but do not remove, the slap's depth.

## Triangle meshes

`Shape::new_mesh` checks every vertex, referenced or not, against `MAX_SHAPE_EXTENT`, and every
index against the vertex count, because Jolt's clean-up reads `vertices[index]` without a check
(`MeshShapeSettings::Sanitize`, `Collision/Shape/MeshShape.cpp:94-111`). Vertex and triangle counts
are at most `i32::MAX`, the index type of Jolt's clean-up and edge search. A mesh's local bounds
are those of its referenced vertices, so the extent bound applies to the vertices themselves.

Jolt stores the vertices quantized to 21 bits over the bounds of the triangles it keeps, with one
step per axis (`TriangleCodecIndexed8BitPackSOA4Flags`), and, when it collides a triangle, scales it
and transforms it into the convex shape's space in `f32` (`CollideConvexVsTriangles::Collide`). Its
collision then asserts when the triangle's `f32` cross product `(v1 - v0) × (v2 - v0)` has a
squared length of at most 1e-12, a length of 1e-6 (`IsNearZero` in
`Geometry/EPAPenetrationDepth.h:113`; Jolt's comment there blames slivers). Jolt's own clean-up drops
triangles below that limit in the mesh's space (`IndexedTriangle::IsDegenerate`), but the cross
product it collides with is formed again in the convex shape's space. The constructor therefore
drops a triangle, and reports it in `DroppedTriangles`, unless twice its area is at least
`1.000001e-6 + 2 · Δ`: Jolt's limit raised by a relative 1e-6, about eight times the rounding of
Jolt's `f32` squared length, where `Δ` bounds how much the cross product can shrink:

- each corner can move by the quantization step on each axis (the bounds' side on that axis over
  `2^21 - 1`), plus `4 · FLT_EPSILON` times the triangle's largest distance from the shape origin
  (the `f32` rounding of the transform), plus `FLT_EPSILON` times the convex extent and the
  triangle's longest edge (the rounding of the result in the convex shape's space, see
  [Convex shapes against meshes](#convex-shapes-against-meshes)); an edge moves by twice that;
- with edges `ab`, `ac` from one corner moved by `e1`, `e2`, the cross product's length is at
  least its component along the unmoved unit normal `n`, which changes by `e2 · (n × ab) + e1 ·
  (ac × n) + n · (e1 × e2)`. The first two terms are bounded per axis, the third by `|e1| |e2|`.
  The cross product is the same from every corner, so the smallest of the three corners' bounds
  holds, and the rule does not depend on the order of a triangle's corners. Jolt's `f32` cross
  product adds `2 · FLT_EPSILON · |ab| |ac|` for the corner where that is largest.

Moves within the triangle's plane across an edge shrink it; moves along an edge or out of the plane
do not, so a thin strip keeps its width wherever the quantization along its narrow direction is
fine. With the default convex extent a right triangle with legs of 1.17 mm near the origin is kept
and one with legs of 1.16 mm dropped; without a convex shape to round in (an extent of 0) legs of
1.001 mm are kept and 1 mm, Jolt's own limit, dropped. A strip 1 m long near the origin, made of two
triangles split along either diagonal, is kept from 0.15 mm wide along the axes and from 0.26 mm in
the worst of 2000 seeded orientations; a single triangle with its apex at mid-length needs the same. In a level mesh with one triangle 1500 m out along x, the x step is 0.7 mm and
the z step 5 µm: a 1 m by 2 mm strip is kept when its 2 mm lie along z and dropped when they lie
along x. Triangles that fail the rule without any quantization are left out of the bounds first;
they are dropped either way, and a far degenerate triangle then does not coarsen the grid for the
others.

Every triangle that survives keeps the same rule under Jolt's actual bounds, which are those of the
surviving triangles and so no larger, so Jolt's own clean-up only drops duplicates (keeping one
copy). The seeded stress (`tests/shape_stress.rs`), which found `EPAPenetrationDepth.h:113` with
sliver soups, includes level-like grids carrying strips 1 µm to 1 cm wide next to far triangles
along x, along z or below (a quarter of them at the origin, where the thinnest strips are kept),
drops its probes onto every such mesh and overlaps it with a box as large as the default convex
extent, the mesh near one of the box's bottom corners. Without the convex shape's term it hits the
assertion at its normal size.

The floor is Jolt's limit and not more. In an asserts build the smallest right triangle kept for a
2 m extent (legs 1.001 mm at the mesh origin, 1.16 mm 70 m out) and for the default one collide with
boxes up to their extent in three orientations, and with boxes up to ten times larger than a 110 m
extent in 40 seeded orientations each
(`the_smallest_kept_triangles_collide_with_convex_shapes_up_to_the_extent`). With the floor lowered
to 0.8e-6 the smallest triangle kept for 200 m (legs 1.003 mm) trips `EPAPenetrationDepth.h:113`
under a box of half extent 100 m; at 0.9e-6 and above nothing asserts there, because the margin
`2 · Δ` still covers it.

## Convex shapes against meshes

Jolt collides a triangle in the convex shape's centre-of-mass space
(`CollideConvexVsTriangles.cpp:43-48`) and goes on only with triangles whose bounds overlap the
convex shape's local bounds grown by the query's separation distance. There every coordinate is at
most the convex extent `E` plus the triangle's longest edge, and the `f32` transform rounds it by
up to half an ulp: a sliver 1 m long and 14 µm wide near the mesh origin keeps its width in its
own space and loses it under a box of half extent 300 m, where coordinates near 300 have an ulp of
31 µm. Half an ulp is at most `FLT_EPSILON / 2` of the coordinate, so each corner moves by at most
`FLT_EPSILON / 2 · (E + longest edge)` on each of the convex shape's axes, and by at most `√3` times
that, under `FLT_EPSILON · (E + longest edge)`, on each of the mesh's. The triangle rule counts
that per corner, with `E` from `MeshSettings::max_convex_extent`: the largest absolute coordinate of a convex shape's local
bounds (relative to its centre of mass, after scaling) plus the separation distance of a collide
query. Bodies collide with the world's speculative contact distance
(`WorldSettings::speculative_contact_distance`, 0.02 m by default, [contact distances](#contact-distances)),
characters with their predictive contact distance; compound children count one by one. The sphere is collided in the
mesh's space instead (`CollideSphereVsTriangles`), which the rule covers too. Shape casts keep the
triangle in the mesh's space (`CastConvexVsTriangles`).

The default `E = 300 m` is the largest round extent at which every prop of the real models tested
in CI loses under 0.1 % of its area ([real meshes](real-meshes.md)). The radio, whose bevels are
strips 1 mm wide, loses 0.021 % at 200 m, 0.024 % at 300 m, 0.17 % at 400 m and 0.69 % at 1100 m.
The default trades reach for thin triangles: a convex shape whose bounds reach beyond 300 m from its
centre (a box of half extent above 300 m, or for a body above 300 m less the world's speculative
contact distance, 299.98 m with the default) can meet a triangle the rule kept for 300 m and trip `EPAPenetrationDepth.h:113`
in an asserts build, or get a distorted contact in a release one. The sweeps below found no
assertion up to seven times a mesh's extent and found some at ten times. A mesh that must collide
with larger shapes is built with a larger `max_convex_extent`, up to `2 · MAX_SHAPE_EXTENT`, and
keeps only thicker triangles: a strip 1 m long and 1 mm wide near the mesh origin is kept up to
about 2080 m along the axes and up to about 1200 m turned the worst of 2000 seeded ways. A mesh
of small detailed objects, which meets only small bodies, keeps more with a smaller extent: the
ScatteringSkull at its own 0.25 m loses 0.48 % of its area at 2 m against 4.2 % at the default.
Near the origin the thinnest 1 m strip kept is:

| `E` | along the axes | worst orientation |
|---|---|---|
| 200 m | 0.099 mm | 0.17 mm |
| 300 m (default) | 0.15 mm | 0.26 mm |
| 750 m | 0.36 mm | 0.63 mm |
| 1100 m | 0.53 mm | 0.92 mm |
| 2000 m | 0.96 mm | 1.7 mm |
| 4000 m | 1.9 mm | 3.3 mm |

A mesh keeps the extent it was built with, and `Shape::new_scaled` checks its triangles for that
extent.

In an asserts build the thinnest sliver the rule keeps (to 1 %) rests under boxes of half extent
200 and 300 m with the default, 300 and 1100 m with an extent of 1100 m and 1500 and 2000 m with
an extent of 2000 m, queried in three orientations and as a heavy body
(`the_thinnest_kept_slivers_collide_with_convex_shapes_up_to_the_extent`).
With the convex term scaled by 0.25 or 0.1 Jolt finds no contact with the sliver in that test, and
at 0 it asserts. In seeded sweeps of the thinnest kept slivers lying along a convex shape's axes,
about 35 000 cases with convex shapes as large as an extent of 1100 m and 1.8 and 3 times larger
asserted nothing. With a mesh built for 110 m, convex shapes 2, 3, 5 and 7 times larger asserted
nothing in 10 000 cases each, and 10 times larger asserted in 9 of 10 seeds of 2000 cases: the
margin is about 7. A convex shape's extent stops at 2000 m plus a separation distance of at most
2000 m, so against a mesh of the default extent a shape up to 20 times larger can be built.

## Scaled shapes

`Shape::new_scaled` adds no numeric bound of its own on the scale. Jolt checks the rest
(`Shape::IsValidScale`, `ScaleHelpers.h`): every component at least `1e-6` in absolute value, uniform
within a squared tolerance of `1e-8` for spheres, capsules and tapered capsules (an absolute
tolerance, so very small scales count as uniform), uniform in X and Z for cylinders and tapered
cylinders, and for a compound a scale each rotated child can take on its own axes. Jolt lets NaN and
infinity through, so the constructor checks that the components are finite first.

What the scale can break is checked on the result: the scaled bounds against `MAX_SHAPE_EXTENT`,
the scaled centre of mass (bounds are relative to it, and a compound's centre of mass moves with the
scale) against the same bound, and every stored triangle of a mesh or heightfield inside the shape
against the triangle rule of [Triangle meshes](#triangle-meshes), for the convex extent the mesh was
built with (the default for a heightfield) and without its quantization term (the stored triangles
are quantized already). A refusal is `ShapeError::ThinTriangles`, which names the scale and the
extent. The coordinates
checked are the ones Jolt rounds: a mesh's own stored coordinates times the scale accumulated above
it, turned by the rotations above it. Jolt
folds compound child positions, rotated-translated positions and centre-of-mass offsets into the
transform it applies afterwards (`ScaledShape`, `RotatedTranslatedShape`, `OffsetCenterOfMassShape`
and `CompoundShape` collision dispatch), so they move a triangle without changing its rounding: a
mesh built 1000 m from its own origin keeps the rounding of 1000 m coordinates however far an offset
moves its centre of mass, and a small mesh at its origin keeps its precision when the centre of mass
is far away. Mass and inertia scale with the shape and go through the mass range and the
[rigid body inertia](#rigid-body-inertia) floor when a moving body is created; a static body
computes no mass. A diagonal inertia must have positive moments: Jolt's `MassProperties::Scale`
rebuilds the diagonal from differences that can round below zero for thin shapes.

## Tapered shapes

A tapered capsule is a sphere when Jolt's `TaperedCapsuleShapeSettings::IsSphere` holds,
`max(t, b) >= 2h + min(t, b)` (`Collision/Shape/TaperedCapsuleShape.cpp:32-35`); the constructor
evaluates it in `f32` exactly as Jolt does (`2h` is exact, so one rounding each side) and refuses
it. Jolt then computes `sinα = d / ((h + d/2) - (-h + d/2))` with `d = b - t`
(`TaperedCapsuleShape.cpp:117-124`) and asserts `|sinα| <= 1`. With `|d/2| <= h` the denominator is at
least `2h (1 - 3u)` and the quotient grows by at most `(1 + u)`, `u = 2^-24`, so any
`|d| <= 2h (1 - 4u)` keeps `|sinα| <= 1`; the constructor requires `|d| <= 2h (1 - 2^-21)`, which is
`2h (1 - 8u)`.

A tapered cylinder's centre of mass divides by `t² + t·b + b²` (`TaperedCylinderShape.cpp:108-123`).
The constructor requires the larger radius to be at least `2^-63` m, about 1.08e-19 m, so that sum
is at least `2^-126`, a normal `f32`. Equal radii make Jolt build a plain cylinder; the constructor
refuses them and points to the cylinder constructor. One radius may be 0, a cone. The shape's
bounds count from its centre of mass, a quarter of the height above a cone's base, so a cone may be
half as tall as `2 · MAX_SHAPE_EXTENT`.

Three seeded stress runs of 10 000 cases each around both boundaries (tapers within 1e-8 of the sphere
case, radius ratios down to 1e-6, larger radii down to 1e-20 m) created, dropped and stepped them
without an assertion.

## Shape binary state

`Shape::restore_binary_state` reads bytes Jolt does not validate. Jolt's restore indexes its table of
shape constructors with the type byte without a range check (`Shape.h:177`), writes as many
compound children as the record names with only an assertion (`CompoundShape.cpp:362-367`),
reads a decorator's child without checking it exists (`DecoratedShape.cpp:68-72`), lets child ids
point forward to form cycles (`Shape.cpp:177-178`) and creates materials through its RTTI factory
from a hash in the stream (`StreamUtils.h:34-42`). The joltc extension therefore walks the graph
itself and checks, before Jolt reads a record: every length inside the data, the type among the
sixteen kinds it saves and equal to Jolt's first byte, child indices naming earlier records,
material indices naming existing materials; after Jolt read it: exactly its bytes read, one child
for a decorator, the compound's own sub-shape count, one material for a convex shape or a plane.
Materials are rebuilt from their kind and user data, never through the factory. An empty shape
keeps its centre of mass, which Jolt's `EmptyShape` does not save.

What stays unchecked is the inside of Jolt's records: array lengths that Jolt resizes to before
reading (`StreamIn.h:43-53`), mesh tree offsets (`NodeCodecQuadTreeHalfFloat.h:231-318`), hull face
and vertex indices, static compound nodes and heightfield block sizes. The checksum detects
common damage but proves nothing about these, so the contract of the `unsafe` restore is that the
bytes are the unchanged output of a save by the same build.

The saved bytes reach Rust as `Vec<u8>`, so every one of them must have a value. Jolt grows its
byte buffers without initialising them (`Array.h:197-206`, `ByteBuffer::Allocate`), so the
sixteen kinds' `SaveBinaryState` were read for bytes no code writes: the mesh tree's node, block,
index and vertex records are written whole and its header's padding is zero-initialised; the
heightfield's sample and edge buffers are cleared before they are filled and every range block is
written; hull points, faces and planes and static compound nodes have no padding and every field
written; mutable compound bounds are written per block of four. None was found, so no
zero-filling allocator is installed; `tests/shape_binary_state.rs` checks that two processes save
equal bytes and that a restored shape saves the bytes it came from.

## Solver step counts

`WorldSettings::velocity_steps` is between `WorldSettings::MIN_VELOCITY_STEPS` (2) and
`WorldSettings::MAX_SOLVER_STEPS` (255), `position_steps` between 0 and 255.

- At least 2 velocity steps: Jolt applies friction with the normal impulse of the previous velocity
  iteration, so with one iteration friction does nothing (`PhysicsSettings.h:83`).
- At most 255 of either: Jolt keeps step counts in 8 bits. The island builder stores each island's
  position step count as `uint8` (`IslandBuilder.cpp:385`), the step statistics cast both counts to
  `uint8` (`PhysicsSystem.cpp:1545-1546`), and the per-body overrides assert `< 256`
  (`Body/MotionProperties.h:192,196`). A larger count would wrap.
- 0 position steps is Jolt's own way to turn the position pass off.

The counts and the two [contact distances](#contact-distances) are written once, in
`PhysicsWorld::new`, through joltc's `JPH_PhysicsSystem_SetPhysicsSettings`. joltc copies every
field of its `JPH_PhysicsSettings` both
ways; the one Jolt field it lacks, `mInternalEdgeRemovalVertexToleranceSq`, is set to Jolt's default,
which oxijolt never changes. `world_settings_leave_every_other_physics_setting_at_jolts_default` checks
that every other field still holds Jolt's default after the write. The bounds are checked
by `step_counts_outside_jolts_range_are_refused` and `step_counts_read_back_from_jolt`; a five-cube
stack at 2 velocity and 0 position steps stays finite and above the floor for 60 steps
(`a_stack_without_position_steps_stays_finite`).

## Contact distances

`WorldSettings::penetration_slop` and `speculative_contact_distance` are between 0 and
`MAX_SHAPE_EXTENT` (2000 m), the rule of `CollideShape::max_separation_distance`, which is the same
Jolt quantity. Both default to Jolt's 0.02 m and are written once, in `PhysicsWorld::new`.

What Jolt does with them:
- The slop is added to a contact's separation, and the solver's position pass corrects only the
  separation that stays negative (`ContactConstraintManager.cpp:1904-1909`). It is a tolerance, not
  a target depth: an overlap within it is left as it is, and with 0 position steps nothing is
  corrected. It also caps how far a linear-cast body may move into another
  (`PhysicsSystem.cpp:1671`).
- The speculative distance widens the broad-phase pairing (`PhysicsSystem.cpp:967`), is the
  narrow phase's maximum separation for pairs without a sensor (`:1101`) and widens manifold
  building (`:1212`, `:1297`). Being that separation, it counts toward the convex extent a mesh is
  built for ([convex shapes against meshes](#convex-shapes-against-meshes)).

`contact_tuning.rs` measures both. A sphere of radius 0.5 dropped from 3 m and 5 m (centre height)
peaks above 0.03 m inside the floor; it comes to rest at about 0.02 m with the default slop and
within 0.005 m + 1e-4 m with a slop of 0.005 (`a_deep_impact_is_corrected_down_to_the_slop`). Placed
0.01 m above the floor, it rests less than 0.005 m deep with the default slop
(`a_gently_placed_sphere_rests_within_the_slop_not_at_it`). The 1e-4 m is an allowance for these
fixtures. A sphere 0.05 m above the floor gets an added contact with a depth of about -0.05 m in its
first step with a speculative distance of 0.1 m and none with 0.02 m
(`speculative_contacts_reach_bodies_within_the_distance`).

At the bounds, two cube stacks and a sphere complete 120 steps with finite state for the six
combinations of 0, 0.02 and 2000 m that
`contact_distances_at_their_bounds_keep_a_small_scene_finite` runs, and two spheres 1998 m apart
with a speculative distance of 2000 m get their far contact and stay finite over steps of 1/60 s and
1 ms (`far_bodies_within_the_speculative_distance_stay_finite`). Both tests also pass with Jolt's
asserts on. They cover these fixtures, not arbitrary scenes. A large speculative distance pairs many
bodies, so size `WorldSettings::max_body_pairs` and `max_contact_constraints` for it
([capacity](#contact-constraint-capacity)).

## Accelerations

`MAX_ACCELERATION` is `MAX_LINEAR_VELOCITY / PhysicsWorld::MIN_DELTA_TIME`, about 5e8 m/s². A larger
acceleration already reaches Jolt's speed clamp within every collision step the world accepts, so
the bound removes no motion that the clamp keeps. `MAX_ANGULAR_ACCELERATION` is the same for Jolt's
angular speed clamp.

## Velocities at creation

Jolt asserts `Length() <= mMaxLinearVelocity` (and the angular counterpart) when it creates a body
(`Body.cpp:424`, `MotionProperties.h:48`). The checks compare Jolt's own `Vec3::Length`, called
through joltc, with Jolt's defaults; a test creates bodies on the bound in 64 directions, which the
asserts leg runs.

`MAX_LINEAR_VELOCITY` is Jolt's default `BodyCreationSettings::mMaxLinearVelocity`
(`BodyCreationSettings.h:111`), `MAX_ANGULAR_VELOCITY` its default `mMaxAngularVelocity`
(`BodyCreationSettings.h:112`).

## Integration

Each step Jolt adds gravity times the gravity factor and the accumulated force times the inverse
mass to the velocity, then asserts that the squared speed is finite (`MotionProperties.inl:26-28`)
before clamping it. Before the clamp `|v| <= 500 + (|gf|·|g| + |F|/m)·dt <= 500 + 1000·5e8 + 2·5e8`,
about 5e11 m/s, where the factor 2 also covers a vehicle's gravity force on its chassis. Its square,
about 2.5e23, is finite. The angular velocity is bounded the same way with
`MAX_ANGULAR_ACCELERATION` and the largest principal inverse inertia.

## Force and torque accumulation

With `mass <= MAX_MASS` an accepted accumulated force is at most `MAX_ACCELERATION · MAX_MASS`,
about 5e14 N, so Jolt's `f32` sums (`Body.h:183,191`) and `mInvMass * F`
(`MotionProperties.inl:134`) cannot overflow. For a force at a point the lever must be finite in
`f32` and every product `lever_i · force_j` at most 1e37, so Jolt's cross product
(`Body.inl:127-131`) stays finite even when its two products cancel. The torque rule counts the
rounding of that cross product, as for [impulses](#impulses).

## Impulses

Jolt applies an impulse at once: `AddImpulse` sets the velocity to `v + J · invM`, and
`AddAngularImpulse` to `w + I⁻¹(R) · L`, through `SetLinearVelocityClamped` and
`SetAngularVelocityClamped` (`Body.inl:133-151`), which mask the locked axes, assert only that the
squared speed is finite and clamp it to the body's maximum (`MotionProperties.inl:23-39`). The body
interface applies them to dynamic bodies only and wakes them (`BodyInterface.cpp:764-808`), so
nothing accumulates: each impulse starts from a velocity within the bounds.

`BodyMut::add_impulse` accepts `|J| · invM <= MAX_VELOCITY_CHANGE`, twice `MAX_LINEAR_VELOCITY`, so
that one impulse can reverse a body at full speed; the speed after it is below 1500 m/s before the
clamp, and its square is finite. `add_angular_impulse` accepts `|L|` times the largest principal
inverse inertia up to `MAX_ANGULAR_VELOCITY_CHANGE`, twice `MAX_ANGULAR_VELOCITY`, a bound on the
masked `|I⁻¹(R) · L|` for any rotation. `add_impulse_at_point` applies both rules, with the angular
impulse `(p - com) × J` that Jolt computes in `f32`; the lever and the products of that cross
product follow the rule of [force and torque accumulation](#force-and-torque-accumulation).

The angular rule takes the largest angular impulse Jolt's `f32` cross product can produce, not the
exact one. Component `i` is `a · b - c · d` (`a`, `c` lever components, `b`, `d` impulse
components). Jolt's default build lets the compiler fuse one product into the subtraction
(`/fp:fast` with MSVC, `-ffp-contract=fast` with GCC and Clang, `Build/CMakeLists.txt:209, 263-268`);
the cross-platform deterministic build rounds both products and the difference. With the `f32` unit
roundoff `u = 2⁻²⁴`, each rounding changes a value `x` by at most `u · |x|`, or by at most
`f32::MIN_POSITIVE` when the result is that small (subnormal or flushed to zero). For the exact
value `L = ab - cd` and `S = |ab| + |cd|` both orders give

    |L_jolt - L| <= u · |L| + u · (1 + u) · S + 3 · MIN_POSITIVE,

so the check adds `u · (|L| + 2S) + 4 · MIN_POSITIVE` to each component's magnitude (the spare
`u · S` also covers the `f64` arithmetic of the check). For a lever exactly parallel to the impulse
`L` is 0, but the fused form keeps the rounding of one product. On a needle of `MAX_MASS` 2 km
long and 0.2 nm thick, the impulse (0, 7e8, 7e8) N·s at the lever (0, 7000, 7000) m has an exact
angular impulse of 0, while Jolt's is 157696 N·m·s (the rounding of `7000 · 7e8`); times the needle's inverse
inertia of about 1.5e14 it overflowed the squared angular speed (`MotionProperties.inl:38`), and
needles 20 nm and 2 µm thick spun up to the clamp. With the bound, the largest accepted impulse
exactly along the lever changed the angular velocity by at most 15.3 rad/s on needles from 0.2 nm
to 0.2 m thick and from `MIN_MASS` to `MAX_MASS`, as one product's rounding is at most `u · S / 2`;
an impulse parallel only up to the rounding of its components reached 35 to 38 rad/s. Both stay
below `MAX_ANGULAR_VELOCITY_CHANGE`. The margin is at most `3u · S`, about 2e-7 of the
products, so it changes the outcome only where `|lever| · |J|` times the largest inverse inertia
is above about 1e8: a lever many orders longer than the body's radius of gyration about the axis
the impulse turns it.

Jolt's angular path multiplies `Rᵀ · L` before the inverse inertia, so `|L|` itself must stay finite
in `f32`. The largest accepted angular impulse belongs to the body with the smallest largest inverse
inertia: a cube of 2000 m half extent and `MAX_MASS` has a moment of about 2.7e12 kg·m², so `|L|` up
to about 2.5e14 N·m·s, far below the `f32` range. A body without rotational degrees of freedom has a
zero inverse inertia, and Jolt masks the whole angular impulse before it multiplies.

## Buoyancy

`BodyMut::apply_buoyancy_impulse` calls Jolt's volume overload of `Body::ApplyBuoyancyImpulse`
(`Body/Body.cpp:196-283`) through two extension functions: one reads the total volume `V`, the
submerged volume `Vs` and the centre of buoyancy `r` (relative to the centre of mass) that Jolt
computes for the surface, the other applies the impulse with exactly those values, so the check sees
what Jolt will use. Jolt computes, all in `f32`:

- the fluid density `ρ = b / (V · invM)` (`b` the buoyancy factor);
- the buoyant impulse `Jb = -ρ · Vs · gf · g · dt` (`gf` the body's gravity factor);
- the relative velocity `vrel = vf - (v + ω × r)` and, when `‖vrel‖² > 1e-12`, the area
  `A = abs(R⁻¹ vrel) · q / ‖vrel‖` with `q = (sy·sz, sz·sx, sx·sy)` from the size `s` of the shape's
  local bounding box; Cauchy-Schwarz gives `A <= ‖q‖`;
- the drag `Jd = (0.5 · ρ · Cd · A · dt) · vrel · ‖vrel‖`, scaled down so that `‖Jd · invM‖ <= ‖v‖`;
- the angular drag `K · ω` with `K = -Cda · Vs / V · dt · l² / invM`, `l` the mean bounding box
  side, times the world inverse inertia and scaled down to at most `‖ω‖`;
- the angular change `I⁻¹ (r × (Jb + Jd))`.

It then adds both changes with `AddLinearVelocityStep` and `AddAngularVelocityStep`, which neither
clamp nor mask the angular velocity (`MotionProperties.h:228-247`); Jolt clamps only in the next
step's integration (`PhysicsSystem.cpp:1622-1627`), after the solver has used the velocities.

### Product chains

MSVC builds Jolt with `/fp:fast` (`Build/CMakeLists.txt:209`), which may reassociate a
scalar product, and a factor of 0 hides nothing in `f32`: `inf · 0` is NaN. A bound on the exact
product is therefore not enough (gravity factor 0 with `ρ = 8.75e36`, a 20 m box of `MAX_MASS`,
makes Jolt's `-ρ · Vs · 0` NaN while the exact `Jb` is 0). For each chain of factors `f₁ … fₙ` the
rule bounds `P = Π max(1, |fᵢ|)`, divisions entering as reciprocals. `P` bounds every sub-product in
every order and association, also when some factor is 0. A chain rule accepts when `P · (1 + 64u)`
is at most `H = 1e37`, so that sums of up to three bounded terms and Jolt's 3 × 3 products with
entries at most the largest principal inverse inertia `λ` stay below `f32::MAX`. The chains:

- density: `b, 1/V, 1/invM`;
- buoyant impulse: the density chain with `Vs, gf, dt` and the largest component of `g`, alone and
  times `invM`;
- relative velocity: `‖ω‖, ‖r‖`;
- area: `‖vrel‖, ‖q‖`;
- drag: the density chain with `0.5, Cd, ‖q‖, dt, ‖vrel‖, ‖vrel‖, invM`;
- angular drag: `Cda, Vs, 1/V, dt, l, l, 1/invM, ‖ω‖, λ`;
- lever: `λ, ‖r‖` and the impulse bound `Jb + ‖v‖ / invM` (after Jolt's clamp the drag impulse is at
  most `‖v‖ / invM`).

### Squared lengths

Where Jolt squares a length (to compare it or to clamp a velocity), the rule bounds that length by
its value, not by its chain: an upper bound from the triangle and Cauchy-Schwarz inequalities,
computed in `f64` from Jolt's own `f32` inputs, times the same `(1 + 64u)`, at most `H₂ = 1e18`. The
square then stays at most 1e36 and three of them sum below `f32::MAX`. Because every sub-product of
the chain behind such a length is within `H`, Jolt's `f32` value differs from the exact one only by a
relative rounding of a few dozen `u` and by subnormal intermediates, at most about `n · 2⁻¹⁵⁰ · H`
(1e-7) in absolute value. The lengths:

- relative velocity: `‖vf‖ + ‖v‖ + ‖ω‖ · ‖r‖`;
- drag change: `‖Jd · invM‖ <= 0.5 · b / V · Cd · ‖q‖ · dt · ‖vrel‖²`;
- angular drag change: `‖I⁻¹ K ω‖ <= λ · |K| · ‖ω‖`;
- lever: `λ · ‖r‖ · (‖Jb‖ + ‖v‖ / invM)`, with `‖Jb‖ = b · Vs / (V · invM) · abs(gf) · ‖g‖ · dt`;
- new velocity: `2‖ω‖ + lever` and `2‖v‖ + ‖Jb‖ · invM`.

Comparing the whole chain with `H₂` instead would lose the ratios `Vs / V <= 1` and
`invM · (1/invM) = 1` and refuse large calm bodies whose squared lengths are tiny: a box of half
extent 152 m at `MAX_MASS` (376 m at 1e4 kg, 591 m at 1e3 kg) at rest, half under water, whose
angular drag change is 0, and at most 0.03 rad/s at `MAX_ANGULAR_VELOCITY`. Every box within
`MAX_SHAPE_EXTENT` and `MIN_MASS..=MAX_MASS`, at rest or at both velocity bounds, in default water,
is accepted.

The density chain implies `V · invM >= 1e-37`, a normal `f32`, so `ρ` never divides by zero. The
unit tests check each rule at its boundary, the two counterexamples above, NaN in every input, the
boxes above, and replay Jolt's arithmetic in `f32` in source order, reversed and with fused
multiply-adds: for 200 000 seeded inputs, for 4000 accepted inputs with `λ` raised to the largest
value the rules accept, spread over three seeded principal moments under a seeded rotation (Jolt's
world matrix `R · diag(d) · Rᵀ` and its products formed in `f32`), and for 4000 accepted inputs with
the linear or the angular drag coefficient raised to the largest value the rules accept. Every
accepted input stays finite, and the squared lengths reach 1e35.

The rules overlap: the density chain is part of the buoyant and drag chains, and the new-velocity
rules contain the lever and the buoyant velocity change. Switching off one rule at a time, the
replays overflowed without the drag chain, the drag change, the angular drag chain, the angular drag
change or the angular new velocity; raising `H₂` to 1e24 or dropping `λ` from the angular drag change
made them overflow too. The density, relative velocity, area, lever and policy rules are pinned by
their boundary tests (the density chain also by the counterexamples). No test fails with only the
buoyant impulse chains, the relative velocity chain, the lever chain or the linear new velocity
switched off, as later rules cover them; they are kept so that the error names the first product
that would overflow.

### Policy and clamp

Only the buoyant velocity change is bounded by policy, like an impulse's:
`b · Vs / V · abs(gf) · ‖g‖ · dt · (1 + 64u) <= MAX_VELOCITY_CHANGE`, computed in `f64` from Jolt's own
`f32` values. A body fully under water may therefore get up to 1000 m/s from one call, and a body
half under water the same factor's half.

Right after Jolt's call, under the same body lock, both velocities are written back through
`SetLinearVelocityClamped` and `SetAngularVelocityClamped`, which mask the locked axes and clamp to
`MAX_LINEAR_VELOCITY` and `MAX_ANGULAR_VELOCITY` as an impulse does. The solver never sees a
velocity beyond those bounds. The angular change of the lever term has no refusal: it grows with
the lever over the radius of gyration, so small bodies entering water get large spins from ordinary
inputs, and a rule like `MAX_ANGULAR_VELOCITY_CHANGE` would refuse them. The test fixture is a
10 cm rod of 10 g falling at 20 m/s with one end, 4 cm from its centre of mass, in the water. Its
drag impulse, about 2.3 N·s, is clamped by Jolt to the rod's momentum `‖Jd‖ = 0.01 · 20 = 0.2 N·s`;
with the rod's inverse inertia about a transverse axis, about `1.19e5 /(kg·m²)`, that drag turns it
by at most `1.19e5 · 0.04 · 0.2 ≈ 950 rad/s`. This is an analytic upper estimate, not a
measurement; what the test measures is that the rod leaves the call at `MAX_ANGULAR_VELOCITY`
(`velocities_are_clamped_right_after_the_call`).

### Volumes and the centre of buoyancy

Jolt takes the total and submerged volume of a box, capsule, cylinder or tapered shape from its
bounding box (`ConvexShape.cpp:383-445`); spheres and convex hulls use their own, compounds sum their
children. A fully submerged shape gets `Vs = V`. A convex shape fully under water puts its centre
of buoyancy at the centre of mass Jolt passes it, so a convex body by itself gets `r = 0` exactly.
An offset centre of mass decorator passes its inner shape a frame moved back by the offset, and a
compound weights its children's centres by their volume: such a body gets a lever and can turn
even fully under water (`a_fully_submerged_offset_body_turns`). Near a grazing waterline Jolt
divides by a small positive volume difference (`PolyhedronSubmergedVolumeCalculator.h`), and the
rounding of `r` can reach the size of the shape; `r` is therefore read from Jolt and bounded, never
assumed to lie inside the shape.

## Soft body forces

Jolt adds a soft body's accumulated force to every vertex as `F · w / N · dt`
(`SoftBodyMotionProperties.cpp:334`), `w` the vertex's inverse mass and `N` the vertex count. The
accumulated force may give the vertex of the largest inverse mass at most `MAX_ACCELERATION`, and is
at most `MAX_ACCELERATION · MAX_MASS` (5e14 N) whatever the inverse masses, as on a rigid body: a
body whose vertices are all pinned (`w = 0`) cannot collect an unbounded force. Changing a vertex's
inverse mass rechecks the force accumulated in the current step against the new inverse masses, so
unpinning a vertex cannot release a force beyond the bound either.

## Soft body pressure

### What Jolt computes

Before each solver sub-step of `dt` seconds Jolt computes the six-volume `V = Σ (x1 × x2) · x3` over
the faces in `f32`, from the vertex positions about the body origin. When `V > 0` it adds
`w · pressure · dt / V · ((x2 - x1) × (x3 - x1))` to the velocity of each vertex of every face, `w`
the vertex's inverse mass (`SoftBodyMotionProperties.cpp:107-118,291-322`). Nothing bounds `1 / V`.

### Acceptance rule

`PhysicsWorld::create_soft_body` accepts a pressure only when

    pressure · A <= MAX_ACCELERATION / MAX_VERTEX_INVERSE_MASS · V_low

- `V_low` is a lower bound of the six-volume Jolt computes before the first step (below).
- `A` is the largest sum, over the faces of one vertex, of an upper bound of `|x2 - x1| · |x3 - x1|`
  in Jolt's arithmetic. An edge `e` between positions `pa` and `pb` is bounded by
  `1.0001 · (|e| + 15 u (|pa| + |pb|))`, with `u = 2^-24` and `|pa|`, `|pb|` the distances from the
  body origin.

So the faces must enclose a positive volume, wound counter-clockwise seen from outside. No vertex,
whatever its inverse mass (at most `MAX_VERTEX_INVERSE_MASS`, also after
`SoftBodyMut::set_vertex_inverse_mass`), gains more than `MAX_ACCELERATION · dt` per sub-step from
the pressure at the start geometry. The pressure coefficient and impulses stay finite, so a
kinematic vertex gets `0 · finite`, not `0 · ∞`.

### The volume's lower bound

`V_low` does not replay Jolt's `f32` operations: their order and rounding depend on the build
(SSE4.1 `dpps`, fused multiply-adds, the rotation Jolt bakes into the vertices). It is the
six-volume in `f64` minus a bound of everything those operations can change. With `u = 2^-24` and
`|p|` a vertex's distance from the body origin:
- a rotation within Jolt's normalization tolerance scales the six-volume by `1 ± 4e-5` and distances
  by at most `1 + 2e-5`, and its rounding moves a position by at most `15 u |p|`;
- one face's term `(x1 × x2) · x3` is off by at most `64 u |p1| |p2| |p3|`;
- each addition of the running sum rounds by at most `u` times the partial sum, which grows the
  error by at most `(1 - u)^-N` for `N` faces;
- underflow adds at most `1e-36` per face.

The error grows with `|p|³`, so far from the body origin it can exceed the volume itself and the
body is refused.

### Measurements

A tetrahedron with three 1 m edges at a right corner takes a pressure of up to about 9e4; the ball
of radius 0.5 m in `pressure_at_the_bound_steps_finitely` takes `MAX_SOFT_BODY_PRESSURE`.

`MAX_SOFT_BODY_PRESSURE` itself is measured: Jolt applies `pressure · dt / (6 · volume)` times each
face's area as an impulse (`SoftBodyMotionProperties.cpp:290-312`), and a closed ball of 1 m with
vertex masses at `MIN_MASS` and at the total-mass bound, at this pressure, stepped 600 times on a
floor in the `asserts` build, stays finite (`pressure_at_the_bound_steps_finitely`).

## Rigid body inertia

Jolt decomposes the inertia tensor of every body that is not static with `EigenValueSymmetric` when
it creates the body (`MotionProperties::SetMassProperties`, also for a ragdoll part in
`Ragdoll::Stabilize`), before it checks whether the moments are near zero. A tensor that is not
exactly diagonal comes from a compound child's rotation or position, or from an offset centre of
mass. Its decomposition asserts as the soft body inertia's below does, and the same floor applies:
`PhysicsWorld::create_body`, the inner body of `PhysicsWorld::create_character`, and the parts of
`RagdollSettings::new` and `RagdollSettings::new_stabilized` accept such a tensor only when
`det I / (sum of the principal 2 × 2 minors)`, a lower bound of its smallest principal moment, is at
least `1001 · 8 · u · |I|_F`, about 4.8e-4 of its Frobenius norm. The check reads the `f32` tensor
Jolt computes itself (joltc's `JPH_Shape_GetMassProperties` and `JPH_MassProperties_ScaleToMass`),
so it needs no error term for the tensor. `Stabilize` (`Ragdoll.cpp:133-183`) multiplies each part's
tensor by its new mass over its old, one rounding per element, before it decomposes it, and rebuilds
a parent's tensor from that decomposition with every moment raised to at least the smaller of twice
its largest and its children's sum; neither moves the smallest moment down by more than a few
`u |I|_F`, far inside the margin. An exactly diagonal tensor is decomposed exactly and only needs
invertible moments or Jolt's near-zero fallback. In the asserts build, seeded thin boxes, capsules
and cylinders in rotated compound children, pairs of such children, and offset centres of mass, with
half lengths from 1 nm to 50 m and masses from 1 g to 1000 t, asserted for `λ_min / |I|_F` up to
8.2e-5 under the earlier rule (`det I / |I|_F² >= 1e-5 |I|_F`, skipped for a tensor below Jolt's
near-zero limit), and none of the 5 271 accepted at or above 1e-4 did. Made thinner until this floor
refuses them, 2 703 such shapes were created and stepped without an assertion, and about 2 500 with
the floor divided by 3; divided by 4 one of ten seeded runs asserted, divided by 6 all ten did, so
the margin over Jolt's onset is 3 to 4, as for soft bodies. The floor refuses a square needle box in
a rotated child when it is about 54 times longer than wide, a capsule or cylinder about 47 times
longer than its diameter, also where Jolt decomposed it without an assertion. In the asserts leg,
`rigid_body_inertia_at_its_bound_decomposes` (bodies), `inner_body_inertia_at_its_bound_decomposes`
(character inner bodies) and `stabilized_ragdoll_inertia_at_its_bound_decomposes` (a three-part
chain whose masses `Stabilize` redistributes) create and step seeded rotated slender shapes at the
thinnest thickness the floor accepts and check that one 1e-4 thinner is refused; each aborts with
the floor divided by 6 and passes with it divided by 3.

## Provided inertia

`BodySettings::mass_and_inertia` hands Jolt the tensor as given
(`EOverrideMassProperties::MassAndInertiaProvided`, `BodyCreationSettings.cpp:213-215`), with no
shape to bound it, so `PhysicsWorld::create_body` checks it in `f64` against four rules, after the
mass rule:

1. Every entry is finite and `I[i][j] == I[j][i]`.
2. Floor. A diagonal tensor (every off-diagonal entry exactly 0) is decomposed exactly
   (`EigenValueSymmetric` returns it unchanged), so each diagonal entry must be at least
   `MIN_PROVIDED_INERTIA`, 1e-6 kg·m². Then the moment vector is longer than 1e-6, Jolt's
   near-zero fallback to unit-sphere inertia (`Vec3::IsNearZero` in `MotionProperties.cpp:46-56`)
   never triggers, and every principal inverse inertia is at most 1e6, inside the `√3 · 1e6` that
   [Character weight and push](#character-weight-and-push) assumes. An upper bound alone bounds no
   inverse inertia: `(1e-30, 1, 1)` has a finite tensor and an inverse moment of 1e30. For a tensor
   that is not diagonal, `det I / (sum of the principal 2 × 2 minors)`, a lower bound of its
   smallest principal moment, must be at least `MIN_PROVIDED_INERTIA + 8 · u · |I|_F`
   (`u = 2^-24`): the moment Jolt's `f32` decomposition returns is within the residual bound
   `8 u |I|_F` of [Soft body inertia](#soft-body-inertia) of the exact one. Jolt reads the `f32`
   entries as given, so no further error term applies.
3. Ceiling. Every moment is at most `6 · mass · MAX_SHAPE_EXTENT²`. A body of that mass whose
   points lie within `MAX_SHAPE_EXTENT` of its centre of mass on every axis has the trace
   `2 Σ m |r|² <= 6 · mass · MAX_SHAPE_EXTENT²`, so this keeps a provided tensor within what a
   shape the crate accepts can produce, and every bound derived from masses and extents still
   holds. A diagonal tensor is checked per moment; for another tensor the Frobenius norm, at
   least its largest moment, must be at most the ceiling, which refuses some tensors an exact
   eigenvalue check would accept.
4. A tensor that is not diagonal must pass the rule of [Rigid body inertia](#rigid-body-inertia):
   positive definite, with `det I / minors` at least `1001 · 8 · u · |I|_F`.

`RagdollSettings::new_stabilized` refuses parts with a provided inertia: `Ragdoll::Stabilize`
multiplies the tensor by the part's new mass over its old (`Ragdoll.cpp:130-140`), which can take a
moment below the floor. `BodyMut::set_shape` computes the inertia from the new shape again.

## Soft body inertia

Jolt sums a soft body's inertia tensor about the body origin in `f32` from its vertices
(`SoftBodyMotionProperties::CalculateMassAndInertia`) when the body is created and after every
`SoftBodyMut::set_vertex_inverse_mass`, and decomposes it with `EigenValueSymmetric`, which asserts
that every eigenvector `v` with eigenvalue `λ` satisfies `|M v − λ v|² <= 1e-6 · max(|M v|², λ²)`
(`EigenValueSymmetric.h:88`). A body with a kinematic vertex skips all of this (infinite mass and
inertia). Jacobi's residual `|M v − λ v|` is a multiple `k` of `u |M|_F`, `u = 2^-24`, whatever the
eigenvalue, so the assertion holds for the smallest principal moment `λ_min` when
`λ_min >= 1001 · k · u · |M|_F`. A scalar `f32` emulation of Jolt's decomposition measured `k` at
most 4 over 42 000 tensors (random spectra and rotations, near-axis tilts, and tensors summed from
random point clouds), so `k` is measured, not derived; the bound takes `k = 8`, a ratio
`λ_min / |M|_F` of at least about 4.8e-4. In the asserts build, 4 000 seeded clouds of point masses
moved to the boundary of this rule with its floor divided by 3 (and 4 random rotations each)
decomposed without an assertion, and with the floor divided by 4 Jolt asserted, so the margin over
Jolt's measured onset is 3 to 4. The asserts leg creates bodies at the bound in
`soft_body_inertia_at_its_bound_decomposes` (which aborts with the floor divided by 4). Jolt's
second check, that the decomposition rebuilds each column of `M` to `1e-5` of its length
(`MassProperties.cpp:56`), never fired in the emulation for tensors of point masses: it needs two
moments much smaller than the third, which point masses cannot give (each moment is at most the sum
of the other two). `PhysicsWorld::create_soft_body` and `set_vertex_inverse_mass` check it in `f64`
with `λ_min` bounded from below by `det M / (sum of the principal 2 × 2 minors)`, after subtracting
a bound of how far Jolt's `f32` tensor can be from the `f64` one in Frobenius norm:
`3 γ_{N+3} Σ m |p|²` for the `N`-term sums of terms of at most three roundings, and, when Jolt bakes
a rotation into the vertices (computed with Jolt's rotation matrix in `f64`),
`(1 + √3)(2 δ + δ²) Σ m |p|²` for its rounding of each position by at most `δ |p|`, `δ = 16 u` (the
15 u of the pressure bound with the rotation's slack). A tensor Jolt sums exactly diagonal (no
rotation, every vertex on a coordinate axis) is decomposed exactly; it is accepted as a rigid body's
is, when its moments are all above 1e-30 or it is near zero. The rule refuses a body whose vertices
lie far from its origin compared with their spread: an 11 × 11 cloth of 1 m with 1 kg vertices is
accepted at 11 m from its origin and refused from 12 m (Jolt asserted at 50 m), and a free straight
line of vertices, whose smallest moment is zero. The rule does not know the shape, so it also
refuses free bodies thinner than about 1/70 of their length (ribbons) or with a radius below about
1/130 of their length (tubes, ropes), which Jolt decomposes: centred on the origin, a 2.5 m ribbon
1, 2 or 3 cm wide and a 3 m tube of radius 1 or 2 cm were refused, and Jolt created and stepped them
without an assert at baked rotations of 0, 0.001 and 0.3 rad. A kinematic vertex skips the check.

## Soft body edge length

Jolt only asserts that a rest length is above zero (`SoftBodySharedSettings.cpp:226,377`) and
divides by edge lengths while it solves; `MIN_SOFT_BODY_EDGE_LENGTH` keeps a degenerate edge out of
the solver with a margin.

## Soft body long-range attachments

An LRA rest distance is at most the sum of all edge lengths, below `2³² · 2√3 · MAX_SHAPE_EXTENT`
(about 3e13 m). With the multiplier of `SoftBodyVertexAttributes::long_range_attachment` at most
`MAX_RATIO` it is at most 3e17 m, and Jolt's square of it (`SoftBodyMotionProperties.cpp:695`) stays
below 1e35.

## Soft body compliance

Jolt divides each compliance by the squared sub-step
(`SoftBodyMotionProperties.cpp:371,445,496,577,594`). A collision step is at least
`PhysicsWorld::MIN_DELTA_TIME` (`step_with_collision_steps` refuses shorter ones), so a sub-step is
at least `MIN_DELTA_TIME` divided by
`SoftBodySettings::MAX_ITERATIONS`, 1e-8 s, and `compliance / dt²` is at most `1e20 · 1e16 = 1e36`,
below `f32::MAX`; Jolt's average of two compliances, `0.5 · (c1 + c2)`, stays finite as well. This
proves that the product is finite, not that the solver is stable at every compliance.

## Soft body skinning

`MAX_SKIN_JOINTS` (4096) is crate policy. A skinning call passes one matrix per joint up to the
largest bound joint, and Jolt allocates one 64-byte skin matrix per inverse bind from its temp
allocator with a `uint` byte count (`SoftBodyMotionProperties.cpp:1142-1144`); with one bind per
joint the cap keeps both at 256 KiB per call.

Jolt packs the normal of each skinned constraint into 32 bits: the number of faces that touch the
vertex and whose three vertices are all skinned in the top 8 bits, the start of its face list in the
low 24 (`Skinned::mNormalInfo`, asserted in `SoftBodySharedSettings.cpp:596,602`).
`SoftBodySharedSettingsBuilder::build` counts these faces per skinned vertex as Jolt collects them
and refuses 256 or more at one vertex, or 2^24 or more in total.

Skinned positions stay finite. Let `E = MAX_SHAPE_EXTENT`. Vertex positions and bind positions are
within `E` per axis of the settings frame, so a rigid inverse bind takes a vertex `v` to
`R_B⁻¹ (v − t_B)`, at most `2√3 E` long. The joints a skinning call passes, and the bind pose a new
body starts in, are within `E` per axis of the body's centre of mass, so a joint matrix moves that
point to at most `2√3 E + √3 E = 3√3 E` from it. The weights are normalised in `f64` and add up to
1 within a few `f32` roundings, so their blend is within the same bound up to a relative error of a
few `2^-24`, below `4√3 E` (about 13 900 m), far inside `f32`.

## Vehicle gravity

Jolt adds `gravity / inverse_mass` to the chassis (`VehicleConstraint::OnStep`); the chassis is a
dynamic body, so the force is at most `5e8 · 1e6`, about 5e14 N.

## Character weight and push

A character presses on what it stands on with the impulse `mass · |g| · dt` at the ground contact
point (`CharacterVirtual.cpp:1474-1481`), so the impulse also turns the ground body.
`PhysicsWorld::update_character` accepts at most `MAX_WEIGHT_IMPULSE`, 1e9 N·s, which changes the
linear velocity of a body of the smallest mass by at most 1e12 m/s and the angular velocity of a
body whose principal inverse inertia is at most `√3 · 1e6` by at most about 6e18 rad/s; both squares
are finite (derivation below). Its push impulse is capped at `delta_velocity / inv_effective_mass`
(`CharacterVirtual.cpp:795-811`), whose effective mass includes the body's rotation, so the velocity
change at the contact is at most the relative normal speed whatever the strength.

`MAX_WEIGHT_IMPULSE` in detail. Jolt keeps a body's principal moments of inertia only while their
vector is longer than 1e-6 (`Vec3::IsNearZero` in `MotionProperties.cpp:46-56`) and otherwise uses
the inertia of a sphere of radius 1, an inverse of `2.5 / mass`, at most 2500 for `MIN_MASS`. So a
body whose principal moments are equal has an inverse inertia of at most `√3 · 1e6`, and the ground
contact lies within `√3 · MAX_SHAPE_EXTENT` of its centre of mass. For such a body and every body
with a smaller principal inverse inertia, the angular velocity change is at most
`√3e6 · 3464 · 1e9`, about 6e18 rad/s, whose square is finite; the linear velocity change is at most
`1e9 · 1e3` m/s. Without the bound, a character of `MAX_MASS` at `MAX_ACCELERATION` with a
one-second update (5e14 N·s) on the edge of a 6 cm cube of `MIN_MASS` overflows the cube's squared
angular speed, which Jolt asserts on (`MotionProperties.inl:38`).

## Character contacts

`CharacterContactListener::adjust_body_velocity` reports the velocity of a body as a character sees
it. `BodyVelocity::set_linear_velocity` and `set_angular_velocity` accept a finite vector whose
length, as Jolt computes it, is at most `limits::MAX_LINEAR_VELOCITY` (500 m/s) or
`limits::MAX_ANGULAR_VELOCITY` (about 47 rad/s): the bounds Jolt clamps every body's velocity to.
So a listener cannot tell a character about a body moving faster than a body can. The bounds depend
on nothing of the call, so a value kept from another call stays valid.

Jolt turns the pair into the velocity of the contact point, `v + ω × r` with `r` from the body's
centre of mass to the contact (`CharacterVirtual.cpp:216-223`), and a real body at the bounds gives
the same: a lever of up to `√3 · MAX_SHAPE_EXTENT` makes that some 1.6e5 m/s, which the character's
solver and `ground_velocity` then report. `a_far_lever_adjusted_velocity_stays_finite` puts a
character at the edge of a static box of `MAX_SHAPE_EXTENT`, reports the box as moving at
`MAX_LINEAR_VELOCITY` and spinning at `MAX_ANGULAR_VELOCITY`, and updates it 120 times next to a 1 kg
cube it pushes, with the world stepped in between: every position and velocity stays finite, in the
default and in the asserts build. Without the setters' check a NaN reaches the character's ground
velocity (an asserts-build probe with the check removed reported `ground_velocity` x = NaN), so the
refusal guards the character's state rather than a Jolt assertion.

## Group filter table size

`GroupFilterTable::MAX_SUB_GROUPS` (4096) is an oxijolt bound. Jolt's table stores one bit per pair
of different sub groups, `n (n - 1) / 2` bits, so 4096 sub groups take about 1 MB, and it indexes
the bits with an `int` computed from `n (n - 1) / 2` in 32-bit unsigned arithmetic
(`GroupFilterTable.h:57`, `:66`), which overflows `int` above 65 536 sub groups. Sub-group ids are
checked against the table's size before they reach Jolt, which asserts on an id at or beyond it
(`:52`) and on a pair of equal ids (`:46`); with either check removed, the asserts build stops on that
assertion.

## Springs

Jolt derives a stiffness `k` and damping `c` from every spring (`SpringPart.h:36-55,91-104`). Both
stay at most `MAX_SPRING_COEFFICIENT`: in stiffness mode directly, in frequency mode through an
upper bound of the effective mass. For a ragdoll joint that bound is computed at creation from the
parts' masses and inertias, including Jolt's `Stabilize` (`Ragdoll.cpp:135-185`); see
`SpringSettings`. For a world constraint it is computed at creation from its two bodies: over the
dynamic ones, the larger of the mass and the largest principal moment of inertia
(`PhysicsWorld::create_constraint`); the constraint's spring setters use the same bound. For a
wheel's suspension it is `MAX_MASS`, since Jolt's suspension effective mass is at most the chassis
mass (`VehicleConstraint.cpp:448-451`).

`MAX_SPRING_COEFFICIENT` keeps `c + dt · k` finite (at most 2e30 for `dt <= 1`), so the softness,
bias and effective mass Jolt computes from them stay finite.

## Anti-roll bars

Jolt computes `stiffness · length difference · dt` for each bar (`VehicleConstraint.cpp:289-293`)
and passes it as the bias `b` of the wheel's suspension constraint (`VehicleConstraint.cpp:508`),
whose impulse is `-K⁻¹ (J v + b)` (`AxisConstraintPart.h:300-301`): a velocity term, scaled by an
effective mass that is at most each body's own along the axis. With
`VehicleAntiRollBar::MAX_STIFFNESS` and wheel lengths at most `MAX_SHAPE_EXTENT`, `b` is at most
5e14 m/s, so the velocity change along the suspension axis stays finite and squares finitely.

## Torque curves

Jolt reads an engine's torque curve at the current rpm over the max rpm
(`VehicleEngine::GetTorque`, `VehicleEngine.h:63`), and the rpm stays within `min_rpm..=max_rpm`
(`ClampRPM`), so it reads x between `min_rpm / max_rpm` and 1. `LinearCurve::GetValue`
(`Core/LinearCurve.cpp:25-39`) interpolates between the points around x as
`y1 + (x − x1)·(y2 − y1) / (x2 − x1)`. With any finite points that overflows: in probes through the
safe API, `(−1e30, 0), (1e30, 1e30)` read at 0.125, `(−1e38, 0), (1e38, 10)` and
`(0, −3e38), (1, 3e38)` gave NaN wheel or track speeds within two steps for wheeled vehicles,
motorcycles and tracked vehicles, with and without Jolt's assertions.

Every vehicle kind's engine therefore takes a curve with x within `0..=1`, neighbouring x at least
`MIN_TORQUE_CURVE_SPACING` (1e-3) apart, and y within `0..=MAX_NORMALIZED_TORQUE` (10). The
spacing is measured on the exact values of the `f32` coordinates and may fall short by one `f32`
epsilon (2⁻²³), which covers rounding x within `0..=1` to `f32`: a curve sampled at `i / 1000`
passes.
Then `x − x1` is at most `x2 − x1` (rounding is monotonic), both are at most 1, the divisor is a
normal `f32`, and the result lies between the two y up to rounding, at most `y_max·(1 + 8·2⁻²⁴)`
with `y_max` the largest y. The engine's largest torque is `max_torque · y_max · (1 + 8·2⁻²⁴)`.
Removing the x or the y range lets the curves above through again. The spacing keeps the divisor
away from subnormal values, where rounding `(x − x1)·(y2 − y1)` to a subnormal adds up to half a
unit to the result instead of a relative error: policy for this bound, not a NaN guard. Wheel
friction curves keep their own rule (finite points, increasing x), because their x is a slip ratio
or a slip angle in degrees.

## Track ratios

Jolt keeps a tracked vehicle's two tracks in step after the engine torque and after the
longitudinal impulse of each unbraked wheel (`TrackedVehicleController.cpp:188-208, 291, 406`). With track ratios `L` and `R` of the
same sign (`L·R > 0`, `:194`) it divides `L·ω_r − R·ω_l` by `L·I_r + R·I_l`, otherwise by
`R·I_l − L·I_r`, where `I` are the track inertias. In both cases the two terms of the divisor have
the same sign, so its magnitude is at least the smaller ratio times the sum of the inertias.

`TrackedDriverInput` keeps both ratios within `1/MAX_RATIO..=1` in magnitude, and
`TrackedVehicleSettings` keeps a track inertia within `MIN_TRACK_INERTIA..=MAX_TRACK_INERTIA`. Then
`L·R` is at least 1e-8, far from underflow, and each divisor term is at least 1e-7. Without a
floor on the ratios, equal ratios of 1e-30 make `L·R` underflow to 0, Jolt takes the second branch,
and with equal inertias the divisor is 0: in a probe both track speeds were NaN after the first
step, with and without Jolt's assertions. The upper bound 1 is what the [track drive
envelope](#track-drive-envelope) assumes for the torque a track receives.

## Track drive envelope

The tracked controller turns the engine's torque into track speeds, keeps the two tracks in step
and turns every wheel at its track's speed (`TrackedVehicleController.cpp`). Settings that are each
in range overflowed those steps in probes through the safe API: track inertias of 1e-30 and 2e-30
under a 500 N·m engine gave track speeds of 2e32 and 1e32 rad/s on the second step, and the
synchronisation quotient `(1e32 − 2e32) / 3e-30` made both infinite; a 1e-36 m wheel on a track
driven at a 0.3 m wheel turned at `1200 · 0.3 / 1e-36` rad/s, infinite, and its rotation angle
became NaN. Every term of the settings alone was finite in both.

`TrackedVehicleSettings` keeps the inputs in physical ranges first: track inertia within
`MIN_TRACK_INERTIA..=MAX_TRACK_INERTIA` (1e-3 to 1e6 kg·m²), every wheel's radius within a factor
`MAX_RATIO` of its track's driven wheel, the torque curve as in [torque curves](#torque-curves).
It then bounds how fast the drivetrain can spin each track and every term the step forms from
that speed.

Notation: `r = 1/MAX_RATIO`, the smallest track ratio; `I_l`, `I_r` the track inertias; `d` a
track's differential ratio; `g_min`, `g_max` the smallest and largest gear ratio in magnitude,
forward and reverse; `N` the max rpm; `T` the engine's largest torque; `h` = `MAX_DELTA_TIME`;
`K = 60/2π`. What changes a track's speed `ω` in a step:

- Damping (`:184-185`) and the brakes (`:294-313`) never raise `|ω|`.
- The drive (`:264-287`) adds `d·L·c·g·torque·dt / I` to a track, `c` the clutch friction (at most
  1, `VehicleTransmission.cpp:106-123`), only while the track turns slower than its speed limit
  `rpm / (g·d·L·K) · 1.001` or against it. The rpm is at most `N` and `|L|` at least `r`, and the
  limit is tested before the torque is added. So the drive leaves `|ω|` at most its value before or
  the track's target `τ = λ + Δ`, with the speed limit `λ = 1.001·N / (g_min·d·r·K)` and one
  torque step `Δ = d·g_max·T·h / I`, the overshoot.
- The synchronisation (`:188-208`) keeps `ω_l/I_l + ω_r/I_r` when `L·R > 0`, otherwise
  `ω_l/I_l − ω_r/I_r`, and leaves the speeds in the ratio `L : R`. The weighted speed
  `B = |ω_l|/I_l + |ω_r|/I_r` then equals the magnitude of the kept sum, so it never grows in the
  synchronisation, and right after it
  `|ω_i| = B·|L_i| / (|L_l|/I_l + |L_r|/I_r)`, that is `|ω_j|/I_j ≤ κ_j·B` with
  `κ_j = (1/I_j) / (1/I_j + r/I_i) < 1`.
- Ground contact (`:351-407`) moves a track towards the ground's speed under a wheel over that
  wheel's radius, as far as the friction impulse allows. It is not part of the envelope; the
  [track inertia ratio](#track-inertia-ratio) and the [track mass ratio](#track-mass-ratio) bound
  it.

A step that drives only track `i` thus gives `B' ≤ τ_i/I_i + κ_j·B`, one that drives both
`B' ≤ τ_l/I_l + τ_r/I_r`, one that drives neither `B' ≤ B`. The tracks start at rest, and the
fixed point of the first is `τ_i·(1/I_i + 1/(r·I_j))`, so `B` never exceeds

```text
β = max(τ_l/I_l + τ_r/I_r,  τ_l·(1/I_l + 1/(r·I_r)),  τ_r·(1/I_r + 1/(r·I_l)))
```

and track `i` never turns faster than `Ω_i = β·min(I_i, I_j/r)`, which also covers its target.
`TrackedVehicleSettings` computes these in `f64` and refuses settings for which any of the
following exceeds 1e30, 2²⁸ below `f32::MAX`; Jolt computes them in `f32`, and the headroom covers
the rounding the real-arithmetic bound leaves out.

- The synchronisation quotient (`:197, 204`), at most `(Ω_l + Ω_r) / (r·(I_l + I_r))`, and that
  times the larger inertia (`:198-206`).
- Each `Ω`, and the torque that locks the track, `Ω·I / MIN_DELTA_TIME` (`:301`).
- Each wheel's speed, `Ω` times the driven wheel's radius over the wheel's (`:63`), and its
  rotation over `h` (`:71`).
- Terms of the settings alone: the transmission torque `g_max·T` (`:266`), each differential
  torque `d·g_max·T` and its impulse over `h` (`:282-286`), the brake impulse over the inertia
  (`:311`), the brake torque over the track's smallest wheel radius and its impulse (`:329-336`),
  and per wheel the inertia over the radius and the radius over the inertia (`:395, 405`).

The divisor of the speed limit, `g_min·d·r·K` in `f32`, must also be a normal `f32`, so that Jolt's
limit is the `λ` above; were it 0, the limit would be infinite and the drive would never stop.

For Jolt's tracked defaults on the test tank `λ` is 7.0e5 rad/s, `Ω` 7.0e9 rad/s and the largest
term, the lock torque, 7e16. The tests find the largest accepted max torque by bisection (8.3e14
N·m on tracks of `MIN_TRACK_INERTIA` and twice that, 4.2e18 N·m with a wheel radius ratio of
`MAX_RATIO`, 4.2e17 N·m on the widest torque curve) and drive those vehicles straight, with one
track at `1/MAX_RATIO`, pivoting, backwards and idle, at 1/60 s and at `MAX_DELTA_TIME`, in the air
and on the ground: the tracks reached 2e19 rad/s, the wheels 1e23 rad/s, every value finite. With
the envelope check removed, a 1e34 N·m engine on the light tracks and a 1e37 N·m engine on the
wide-ratio tracks gave NaN on the second step, in the default and the asserts build. With the
inertia range or the radius ratio removed instead, the envelope refuses the two probes above by
itself: those ranges keep its inputs physical.

### Live changes

`VehicleMut::set_engine`, `set_transmission` and `set_tracks` change the drivetrain of a tracked
vehicle that is already moving, and Jolt never clamps a track's speed. After a change the tracks do not start at
rest. The vehicle keeps two bounds over every configuration it had: each track's largest speed
`Ω_old,i`, and the largest weighted speed `β_old`, measured with the inertias `I_old,i` of the
configuration before. With the new inertias `I_i` the weighted speed then starts at most at

```text
B_0 = min(Ω_old,l/I_l + Ω_old,r/I_r,  β_old·max(I_old,l/I_l, I_old,r/I_r))
```

(the first from each track's bound, the second because `|ω_i|/I_i = (|ω_i|/I_old,i)·(I_old,i/I_i)`).
The induction above starts from `B ≤ B_0`, so the new configuration keeps

```text
β' = max(β, B_0)
|ω_i| ≤ Ω_i = max(Ω_old,i, β'·min(I_i, I_j/r))
```

and the envelope is checked with these `Ω_i`; `β'` and `Ω_i` become the stored bounds. A change
the envelope refuses changes nothing. A change that keeps the inertias gives `B_0 ≤ β_old ≤ β'`,
so applying the same configuration again leaves the bounds as they are; the first bound alone
would double `β'` on each such change for tracks of equal inertia. A `WorldState` restore can only
bring back track speeds that some configuration the vehicle had allowed, and the stored bounds
cover every configuration: the track speed bounds are running maxima and never shrink.

## Track inertia ratio

Under each unbraked wheel in contact, Jolt sizes the longitudinal impulse for that track's inertia
alone, applies it, and synchronises the tracks again (`TrackedVehicleController.cpp:394-406`). The
synchronisation keeps `ω_l/I_l + ω_r/I_r`, not the tracks' angular momentum, so the lighter track's
speed sets both: an impulse that changes the light track's speed moves the heavy track by about as
much, with nothing paying for it. With very unequal inertias this diverged in probes through the
safe API. A 4000 kg tank on 50 m wheels with track inertias 1e-3 and 1e6 kg·m², tire and ground
friction 1000 and Jolt's tracked engine at 500 N·m, driving straight at 60 Hz, had tracks at 3e20
rad/s after the first step and NaN tracks and chassis after the second, either way round; the
asserts build stopped at `MotionProperties.inl:28` (`isfinite(len_sq)`). The engine torque made no
difference. With the lighter track at 1e-3 to 10 kg·m², the onset needed a ratio of about 1e3,
combined friction of 300 or more and wheels of 5 m or more.

`TrackedVehicleSettings` refuses a larger track inertia above `MAX_TRACK_INERTIA_RATIO` (100) times
the smaller one. At that ratio, with the lighter track at 1e-3 or 1 kg·m², wheels of 5, 50 and
1000 m and friction 1000, the tank stayed finite for 300 steps driving straight and with ratios,
throttle and brake changing every step, in the default and the asserts build. With the rule
removed, the scene above gives NaN again in the default build and stops at the same assertion in
the asserts build. The ratio is policy from these measurements, not a derived bound.

It does not bound ground contact as a whole: the same solve diverged with equal inertias when
the tracks were heavy for the chassis, which the [track mass ratio](#track-mass-ratio) bounds.

## Track mass ratio

Under each unbraked wheel in contact, Jolt sizes the longitudinal impulse as if the chassis stood
still: the slip `s = ω·r − v` between the track and the ground under the wheel, times the track's
mass at that wheel `M = I/r²` (`I` the track's inertia, `r` the wheel's radius;
`TrackedVehicleController.cpp:394-406`). The impulse also moves the chassis: by `w` per unit of
impulse at that point along the longitudinal direction `d`, with the chassis' inverse effective
mass `w = 1/m + (c × d)ᵀ I⁻¹ (c × d)` (`c` the contact from the centre of mass, `I⁻¹` the chassis'
inverse inertia; `AxisConstraintPart.h`). Leaving the synchronisation of the tracks and the
friction limit aside, the slip after the solve is `−M·w·s`: the solver, which repeats this for
every wheel in every velocity iteration, converges only while `M·w` stays below about 1. The
synchronisation and the friction limit move that onset, so the bound is measured.

`PhysicsWorld::create_tracked_vehicle` computes `M·w` for every wheel in `f64` from the chassis'
inverse mass, principal inverse inertia, inertia rotation and centre of mass, and refuses the
vehicle when any wheel's exceeds `MAX_TRACK_MASS_RATIO` (0.25). `VehicleMut::set_tracks` checks
new tracks the same way, with the chassis' mass properties of creation: a chassis cannot change
its shape or motion type while a vehicle uses it. `w` is the largest the wheel can give:
- `d` is any direction in the wheel's forward/up plane. Jolt's longitudinal direction is the
  ground normal crossed with the wheel's right (`VehicleConstraint.cpp:254-271`), so it lies in
  that plane: along the wheel's forward on ground perpendicular to the wheel's up, turned toward
  the up on an edge, a kerb, a ramp or a wall. The largest `|a × d|` over the plane, measured with
  `I⁻¹`, is the square root of the larger eigenvalue of a 2×2 form.
- Every collision tester puts the contact within `ρ = max_length + √(r² + (width/2)²)` of the
  attachment point `a`: the ray ends `max_length + r` along the suspension, the cast sphere's
  centre stops `max_length + r − radius` along it, the cylinder's centre `max_length` along it with
  its surface within `√(r² + (width/2)²)`. Since `|c × d|` measured with `I⁻¹` is a norm of `c`, it
  is at most `|a × d| + ρ·σ`, with `σ²` the largest principal inverse inertia. The ratio is not
  tied to the tester, which can be replaced after creation.
- With a suspension force point, Jolt pushes there and measures the slip at the contact; the
  mixed term is at most the product of the two levers, so the larger lever is used.

Over the plane the lever takes in the chassis' roll and pitch inertia, so the hull's shape matters
as much as its mass. Tracks on Jolt's tank wheels at x = ±1.7 under a 4000 kg box hull:

| Hull half extents, centre of mass offset | Ratio of 10 kg·m² tracks, along the forward | Over the plane | Heaviest accepted tracks |
|---|---|---|---|
| `TankTest`: 1.7, 0.5, 3.2; −0.5 | 0.070 | 0.226 | 11.0 kg·m² |
| short: 1.7, 0.5, 1.0; −0.5 | 0.201 | 0.641 | 3.9 kg·m² |
| narrow: 0.5, 0.1, 3.2 | 0.079 | 2.09 | 1.2 kg·m² |
| short and narrow: 0.5, 0.1, 1.0 | 0.488 | 2.67 | 0.94 kg·m² |

Measured with the rule removed, through the safe API, in the default build: 60480 runs over the
four hulls at plane ratios of 0.02 to 12, tire friction 4/2 on ground of friction 0.2, 1 and 1000
and tire and ground friction 1000, ray, sphere and cylinder testers, on flat ground, kerbs of 0.3
and 0.6 m, a wall, ramps of 30 and 45° approached from flat ground, a 30° slope facing uphill and
across, 100 kg rubble and a 100 kg slab, driving straight, turning in place and through the gears,
at 1/60 s for 600 steps, 0.1 s for 100 and 1 s for 20, after settling at 1/60 s; then 58000 runs at
1 s on the static scenes at plane ratios of 0.001 to 1.
- At 1/60 and 0.1 s nothing went NaN below a ratio of 8 (short hull at a wall, tire and ground
  friction 1000), and the chassis first reached Jolt's angular velocity clamp (15π rad/s) on static
  ground at game friction at 6 (short narrow hull, 0.1 s). At game friction nothing went NaN up to
  12 at any step length.
- At 1 s with tire friction 4/2 and ground friction 1000, the first NaN was at 6 (short narrow
  hull, 45° ramp).
- At 1 s the onsets are lower and scattered: at game friction the chassis reached the clamp from
  0.32 (short hull parked on a 30° slope) and 0.48 (`TankTest`, 30° ramp); with tire and ground
  friction 1000, 3 of 17280 runs between 0.001 and 0.2 went NaN, the lowest at 0.062 (short narrow
  hull, 30° ramp). The narrow hulls also reach the clamp at 1 s with tracks of 1e-3 kg·m²: that
  part does not come from the tracks.

`MAX_TRACK_MASS_RATIO` is 0.25, the round value just above `TankTest`'s 0.226, 24 times below the
onsets at 1/60 and 0.1 s and at ground friction 1000 with game tires. No ratio that keeps Jolt's
tank covers the onsets at 1 s steps: four times below them would be 0.08 at game friction and 0.016
with tire and ground friction 1000, which refuse `TankTest`'s tracks above 3.5 and 0.7 kg·m².

With it, 24000 accepted runs drove the four hulls with their heaviest accepted tracks and
`TankTest` with tracks of 10 and 11 kg·m² (20, 40 and 70 are refused) over the scenes above plus a
15° ramp and slope, a 0.15 m kerb, 1 and 10 kg rubble, slabs of 1, 10 and 1000 kg and a second tank
across the hull, at 1/240, 1/60, 0.1, 1 and 1e-6 s, through gear changes and inputs that flip every
step. Up to 0.1 s every run on static ground stayed finite and below the clamp at every friction,
and every run at game friction did on dynamic bodies too. At 1 s the narrow hulls reached the clamp
about as often as with tracks of 1e-3 kg·m² in the same scenes, and 7 runs went NaN, all with tire
and ground friction 1000.

Not covered:
- 1 s steps with tire and ground friction 1000 on the short and narrow hulls. Over 50 chassis
  masses from 3900 to 4096 kg, on the static scenes, 0 of 14400 runs went NaN with tracks of
  1e-3 kg·m² and at ratios of 0.02 and 0.05, and 5, 29 and 37 at 0.1, 0.15 and 0.25; `TankTest`
  stayed finite at all of them. The onset depends on the hull, not only on the ratio.
- A dynamic body under the wheels: the contact moves that body too, and its inverse mass adds to
  `w`; the ratio counts the chassis alone. With the rule removed, 100 kg rubble or slabs at 1 s and
  tire and ground friction 1000 gave NaN from a ratio of 0.75. With the rule, at up to 0.1 s and
  ground friction 1000, rubble of 1 and 10 kg and a 1 kg slab brought the chassis to the clamp in up
  to 9 of 384 runs per hull (at most 1 with tracks of 1e-3 kg·m²).

## Motorcycle lean

**Lean angle.** With the lean steering limit on, Jolt limits the steering angle to
`asin(wheel base · tan(max lean angle) · |g| / (v² · cos caster))`
(`MotorcycleController.cpp:149,177`), with an `asin` that clamps its argument. The tangent grows
without bound toward 90°, and the `f32` value of π/2 lies just above 90°, where it is about
−2.3e7: in a probe the front wheel then steered 90° although its largest steering angle was 30°.
`MotorcycleSettings::MAX_LEAN_ANGLE` is 80°, where the tangent is 5.67.

**Lean spring.** While both wheels touch the ground, Jolt applies the angular impulse
`(k·d − c·ω_f + k_i·∫d)·dt` about the chassis' forward axis, minus what it applied earlier in the
step (`MotorcycleController.cpp:214-226`), through `Body::AddAngularImpulse`, which clamps the
angular velocity to `MAX_ANGULAR_VELOCITY` afterwards (`Body.inl:149-154`). `d` is the lean error,
an `acos` within `[0, π]` with a sign, and `ω_f` the angular velocity about forward. The first
solver call of a step applies the whole term (`PreCollide` resets the applied impulse, `:187`).
Later calls apply only the change: the target, the integral and the chassis rotation are fixed
within a step, so the spring part cancels and the damping part changes by at most `c·Δω_f·dt`.
`create_motorcycle` therefore requires

`(k·π + c · MAX_ANGULAR_VELOCITY_CHANGE) · I⁻¹max · (1 + 4·u) ≤ MAX_ANGULAR_ACCELERATION`,

with `I⁻¹max` the chassis' largest principal inverse inertia (an upper bound for any axis) and
`u = 5e-7` the tolerance within which the forward axis is a unit vector, computed in `f64`. Each
call then changes the angular velocity by at most about `2 · MAX_ANGULAR_ACCELERATION · dt`, far
from where its squared length overflows. With `c·dt·I⁻¹max > 1` the damping overshoots within a
step and the clamp bounds it: the rule keeps the values finite, not the controller well behaved.
Jolt's default spring (5000 and 1000) fits a chassis with `I⁻¹max` up to about 428 1/(kg·m²); the
240 kg test motorcycle has about 0.1. In a probe without the rule, `k = c = 1e30` on that chassis
failed Jolt's assertion that the squared angular velocity is finite (`MotionProperties.inl:38`).

**Integration coefficient.** Jolt's `MotorcycleController::SaveState` writes the target lean but
not the integrated lean angle `∫d` (`:263-275`), so `MotorcycleSettings` accepts only an
integration coefficient of 0, Jolt's default; with 0 the integral has no effect.

## Restitution

At most 1, so the restitution target speed is at most the approach speed.

## Friction

At most `MAX_FRICTION`, so Jolt's combined friction `sqrt(f1 · f2)`
(`ContactConstraintManager.h:554`) is finite and the friction impulse bound, combined friction times
the normal impulse (`ContactConstraintManager.cpp:1714-1715`), is never `0 · ∞` = NaN. A cube
sliding on a floor, both at `f32::MAX`, had NaN velocities within three 60 Hz steps.

`MAX_FRICTION` keeps that product finite: any coefficient whose square is finite (below about
1.8e19) does, and 1000 is far above the friction of real materials.

### Tire friction

A tire max impulse callback returns two friction coefficients. The binding clamps each to
`0..=MAX_FRICTION`, NaN to 0, and hands Jolt the coefficient times the suspension impulse as the
tire's impulse cap, the formula of Jolt's default (`WheeledVehicleController.h:191-196`). Without
a callback the coefficient is the wheel's friction curve combined with the ground's friction,
`sqrt(curve · ground)` (`VehicleConstraint.h:238-243`), which Jolt reads nowhere else. A curve
of `MAX_FRICTION` on ground of `MAX_FRICTION` gives exactly `MAX_FRICTION`, so a callback can
give a wheel no grip a vehicle without one cannot have, and the cap is finite for the reason
above.

That much grip does not keep Jolt's solver stable. Jolt sizes each wheel's longitudinal impulse
for the wheel alone, `(ω − v/r)·I/r` (`WheeledVehicleController.cpp:705-718`), as if the chassis
stood still, and bounds it only by the cap, so on wheels heavy against their chassis a large cap
lets the iteration overshoot. Measured through the safe API in the default build: a 1500 kg
chassis on four wheels of radius 0.35 with a ray tester, settled on flat ground and driven from
10 m/s through throttle, full steering, brakes and reverse for 4 s, at 1/60 and 0.1 s. With
friction `MAX_FRICTION` the callback and the curves gave bit-identical runs
(`the_largest_friction_of_a_callback_matches_the_largest_friction_curves`):
- a 1.8 × 0.6 × 4 m box on wheels of 1000 kg·m², front wheels steering 70°: Jolt's 500 m/s clamp
  at tick 227 (1/60 s) and 38 (0.1 s);
- a 1 × 0.6 × 1 m box on wheels of 8.4 kg·m² at (±0.5, −0.3, ±0.75) with suspension force points
  at a height of 1.15 m, above the centre of mass: the 15π rad/s clamp at tick 130 and 36;
- a 0.6 × 4 × 0.6 m box with wheels of 3.84 kg·m² 3 m ahead and behind, front wheels steering 70°:
  at most 12.5 m/s and 9.2 rad/s.

With Jolt's default curves on ground of friction 0.2 none of the three reached a clamp; the
second came closest, 40.6 rad/s at 0.1 s.

Not covered:
- Wheels heavy against their chassis at large friction, from a callback or from curves: no bound
  on the wheel inertia is derived, and the first two cases above reach Jolt's clamps.
- Friction curve values are not bounded: a 1500 kg chassis on wheels of 1000 kg·m² with friction
  1e30 was thrown 395 m up while settling, without any callback.

## Coupling ratios

Jolt multiplies the inverse mass or inertia of body 2 by the ratio's square in the effective mass,
and body 2's velocity (for racks and pulleys also its impulse) by the ratio
(`GearConstraintPart.h:81,122`, `RackAndPinionConstraintPart.h:82,123`,
`IndependentAxisConstraintPart.h:71,112` for pulleys). With a principal inverse inertia of at most
`√3 · 1e6` (see "Character weight and push"), `MAX_RATIO² · I⁻¹` is at most about 1.7e14, far from
`f32` overflow. A ratio of 1e4 already turns a pinion ten thousand radians per metre of its rack;
tests step both bounds on the lightest and heaviest bodies.

Gears get a tighter range, `1..=MAX_GEAR_RATIO`, because of a defect in Jolt's gear solver (Jolt
5.6, unchanged on Jolt's master as of 2026-09-28): `GearConstraintPart::ApplyVelocityStep` and
`SolvePositionConstraint` apply the impulse to body 2 as `λ · I2⁻¹ · b`, without the ratio of the
Jacobian `[a, r·b]`, while the effective mass `1 / (A + r²·B)` includes it squared (`A`, `B` the
inverse inertias about the two axes). Each solver iteration therefore keeps
`1 − (A + r·B) / (A + r²·B)` of the velocity error `ω1 + r · ω2`:
- below 1 it approaches `1 − 1/r` for a light body 2, which is negative: below 1/2, and for every
  negative ratio, its magnitude exceeds 1 and the error grows until the bodies' velocities are not
  finite;
- from 1 up it is in `[0, 1)` but approaches `1 − 1/r` for a heavy body 1, so with Jolt's 10
  velocity iterations per step the gear needs more steps to restore the relation the larger the
  ratio.

The upper bound is measured. `MAX_GEAR_RATIO` is the largest round ratio that, after a disturbance,
brings the error back to within 2 % of its initial value within 10 steps for the mass distributions
measured: worst 1.6 % at ratio 10, 6.5 % at 20, 60 % at 100, and at 1e4 91 % still after 60 steps.
The first step after a disturbance leaves up to `(1 − 1/ratio)^10`, 35 % at ratio 10.
`gear_keeps_its_velocity_relation_at_the_largest_ratio` tests it at the bound.

## Lever-arm ratio

`MAX_LEVER_ARM_RATIO` is measured, not derived. Jolt solves each constraint part with its effective
mass `K = Σ (m⁻¹ · 1 + [r]× I⁻¹ [r]×ᵀ)` (`PointConstraintPart.h`, `AxisConstraintPart.h`) in `f32`.
A ratio of at most `B` per body bounds the lever terms by `B · m⁻¹`, so `K`'s condition number stays
below `1 + B`. Two failures were measured with a body at the velocity bounds, under gravity, for 120
steps, in the `asserts` build:
- a body held far from its centre of mass: a 1 g, 6 cm cube on a hinge 116 m away (a ratio of 4.5e7)
  went to NaN, two 1 kg, 1 m cubes joined by a point 3000 m away (1.1e8) moved erratically and at
  4000 m Jolt asserted that a squared velocity is finite (`MotionProperties.inl:28`), and a cube on
  a static body took angular velocities rounded to powers of two from a ratio of about 1e8;
- two light bodies held rigidly (a fixed constraint, a six-DOF constraint with every axis fixed, a
  swing-twist constraint with zero ranges): their accumulated impulse grows step after step until
  Jolt asserts that the squared angular velocity is finite (`MotionProperties.inl:38`), from
  `|r| / k` of about 37 (a ratio of 2700) for 1 g and 1 kg cubes of 6 cm and 20 cm, for the 6 cm
  cube also at a tenth of the velocity bounds; none of 24 seeded cases failed at `|r| / k` of 34 or
  below. Point, hinge, cone, slider and six-DOF constraints with limited rotations did not fail at
  `|r| / k` of 650.

A derivation in the style of `MAX_WEIGHT_IMPULSE` (products of the largest accepted inverse inertia,
lever and impulse kept finite in `f32`) allows levers of hundreds of metres and does not exclude the
second failure, so the bound is the measured onset divided by 2.7.

## Kinematic drive

Jolt's `MoveKinematic` sets a velocity of `move / dt` without clamping it
(`MotionProperties.inl:9-21`). `BodyMut::move_kinematic` and
`RagdollMut::drive_to_pose_using_kinematics` compute the velocity with Jolt's own operations first
(the `f32` difference of the centre of mass and `Quat::GetAngularVelocity` with its small-angle
branch) and accept the move only when it stays within `MAX_LINEAR_VELOCITY` and
`MAX_ANGULAR_VELOCITY`, the bounds of every other velocity input. The check uses the velocity
before Jolt zeroes the locked axes, which can only make it shorter.

## Skeleton mapper chains

`SkeletonMapper::map` runs Jolt's `SkeletonMapper::Map` (`Jolt/Skeleton/SkeletonMapper.cpp:165-210`).
It turns each chain's start by `Quat::sFromTo(actual, desired)`, where `actual` is the direction
along the animation chain as the local pose lays it out and `desired` the direction between the two
ragdoll joints, and passes the quaternion to `Mat44::sRotation`, which asserts that it is
normalised. `sFromTo` (`Jolt/Math/Quat.inl:206-256`) multiplies the squared lengths and normalises
`(actual × desired, |actual| |desired| + actual · desired)`; for a short non-zero vector the
squares underflow and the result is NaN. The fork's C function `JPH_SkeletonMapper_Map2` refuses
such chains before Jolt runs, with `MIN_MAPPED_CHAIN_LENGTH` (`F` = 1 mm) as the floor, and reports
the ragdoll joint at the chain's end (`RagdollError::DegenerateChain`). Notation: `u = 2^-24`,
`γ_n = n u / (1 − n u)`.

- **Desired** is one `f32` subtraction of two ragdoll pose translations in the guard as in `Map`,
  so the guard sees Jolt's value. It must be exactly zero, component by component (Jolt then gets
  `len = 0` and `w = 0` and returns the identity), or at least `F` long.
- **Actual** is the difference of two translations the guard computes again: of the chain start
  `S = A J` (the ragdoll joint's pose and its mapping) and of `S L_1 ... L_k` (the `k` local
  transforms after the chain start). Each `Mat44` product is a set of 4-term inner products, which
  the compiler may order and fuse differently in the two copies, so the guard bounds Jolt's value
  instead of replaying it. For any order, with or without fused multiply-adds, a computed product
  `fl(X Y)` is within `γ_4 |X| |Y|` of `X Y`, entry by entry (Higham, *Accuracy and Stability of
  Numerical Algorithms*, 2nd ed., §3.1). Bounding products of absolute values along the chain
  would grow without limit for rotations (by about `√2` per link that turns 45°: 9e18 after 128
  links), so the guard carries norms instead. With every matrix written as `[R t; 0 1]`, it keeps
  in `f64` the bounds `ρ >= ‖R‖₂` and `θ >= |t|` of the exact product and `r >= ‖R̂ − R‖_F` and
  `d >= |t̂ − t|` of any computed copy `[R̂ t̂; 0 1]`. A step by a local transform with
  `q >= ‖R_L‖₂` (the root of the largest absolute row sum of `R_Lᵀ R_L`), `f = ‖R_L‖_F` and
  `n = |t_L|` gives, with `φ = √3 ρ + r >= ‖R̂‖_F`,
  `ρ' = ρ q`, `θ' = θ + ρ n`, `r' = r q + γ_4 φ f` and `d' = d + r n + γ_4 (φ n + θ + d)`:
  the old error carried through `L` plus the new product's error, whose Frobenius norm is at
  most `γ_4 ‖R̂‖_F ‖R_L‖_F` for the rotation and `γ_4 (‖R̂‖_F |t_L| + |t̂|)` for the translation.
  The start has `ρ = q_A q_J`, `θ = q_A n_J + n_A`, `r = γ_4 f_A f_J` and
  `d = γ_4 (f_A n_J + n_A)`. A rotation has `q` within a few `u` of 1, so the bounds grow with
  the number of links and not with how the chain turns. Each step also adds
  `2^-121 (1 + φ + f + n)` to `r` and `d`, which covers results and inputs below `FLT_MIN`
  flushed to zero. Either copy's `actual` is then within
  `δ = d_k + d_0 + u (θ_k + d_k + θ_0 + d_0) + 2^-121` of the exact difference, the `u` term for
  the final subtraction. The allowance is `Δ = 2δ × 1.001`; the factor also covers the `f64`
  evaluation of the bounds, whose relative error stays below 1e-5 for any chain a skeleton can
  hold (fewer than 2^31 joints). The guard requires `|actual| − Δ >= F`, so Jolt's `actual` is at least `F` long. `Δ`
  grows with the chain's distance from the pose's root offset and its number of links: about
  0.1 mm for 128 links around the root offset, about 9 mm for two links at 4 km, where
  `degenerate_chains_are_refused` sees a 6 mm chain refused and a 50 cm one accepted.
- **Why 1 mm.** With both lengths at least `F`, `len = sqrt(|a|² |b|²) >= F² (1 − 4u) > 2^-21`. If
  `|a · b| < len / 2`, then `w > len / 2`. Otherwise `len` and `−a · b` are within a factor 2 of
  each other and both above `2^-22`, so their sum is exact (Sterbenz) and, unless zero, a multiple
  of `2^-45`: `w² >= 2^-90`, far above the smallest normal `f32`. `Normalized` then divides by a
  normal length and gives a unit quaternion within a few `u`, inside `IsNormalized`'s `1e-5`. When
  `w` is exactly zero and `len` is not, Jolt takes a normalised perpendicular of `actual`, whose
  two components have a squared sum of at least `|a|² / 2`. Any `F` above `2^-10.5` (about
  0.69 mm) works; 1 mm is the round value above it and leaves room for terms flushed to zero below
  `2^-126`. An exactly zero `actual` would be safe too; the guard refuses it with every other short
  one. The floor makes the turn a unit quaternion, not an accurate one: near opposite directions
  `w` is a difference of nearly equal terms, and on a 2-link chain whose ragdoll direction lies
  `ε` from the exact opposite of its animation direction the turned chain missed its target by
  0.77° at `ε` = 3e-5 rad, 24° at 1e-6 rad and by an arbitrary angle below 2e-7 rad, with no NaN
  and no assert.
- **Upper bounds.** The guard also requires `|actual| + Δ <= 1e18` and `|desired| <= 1e18`, so
  each squared length stays below 1e36 (`f32::MAX` is 3.4e38), and
  `(|actual| + Δ) |desired| <= 4e18`, so the product of the squared lengths (at most 1.6e37) and
  the squared quaternion (at most 8e37) stay below `f32::MAX`. The length bound matters on its own
  when `desired` is exactly zero: Jolt still squares `actual`, and an infinite square times zero
  is NaN. Separately from these direction bounds, the sum of absolute terms of every entry the
  chain's products compute, `φ f` and `φ n + θ + d` per step (`f_A f_J` and `f_A n_J + n_A` at
  the start), must be at most 1e30, which keeps every `f32` product and partial sum of either copy
  finite. The safe API's inputs stay far below all of these: local translation components within
  `MAX_POSITION` over at most 1023 links give `|actual| <= 1.8e10` m and model translations within
  `2 MAX_POSITION` give `|desired| <= 6.9e7` m with the `double-precision` feature (product
  1.2e18), about 2.5e11 with `f32` positions, and rotations keep the norm bounds near those
  lengths.
- **Rotations.** Every rotation the mapper passes to Jolt is normalised first: Rust's unit check
  sums the squares left to right and Jolt's `Vec4::LengthSq` pairwise, so a quaternion can pass one
  tolerance and fail the other (the unit tests use one at 1.0000099 and 1.00001).

An `f32` replay of the guard against four evaluation orders of Jolt's product (left to right,
pairwise, and fused in both directions), for 1 to 1023 links with links turning up to 180°, chain
starts up to 2e7 m from the root offset, and 128- and 1023-link chains turning 45° per link about
their axis (with links along it and beside it), found the largest difference at 2.1 % of the
allowance and a unit quaternion for every accepted chain; the turning chains near the root offset
are accepted. The replay fails with the allowance removed, and refuses the turning chains when the
spectral bound `q` is replaced by the Frobenius norm. It was run outside the test suite and is not
kept in the repository. With the floor removed from the C function, the raw test's 1e-30 m
direction came out of `Map` as NaN rotations. Jolt is built with `/fp:fast` on MSVC, and there
`Mat44::sRotation`'s `IsNormalized` assert did not fire for the NaN quaternion, so an asserts build
does not catch it either.

## Skeleton mapper neutral poses

`SkeletonMapper::new` re-expresses the animation's neutral pose relative to the ragdoll neutral
pose's root offset: `(o − r) + t` per component, with both root offsets and every world position
`o + t` within `MAX_POSITION` (the poses' own check). Exactly, that is a difference of two positions
in the frame, at most `2 MAX_POSITION`. The computed value rounds twice in `Real` and once to `f32`
and the world position it was checked through rounded too, so it is at most
`2 MAX_POSITION (1 + 2^-22)` and finite, and can exceed `2 MAX_POSITION` by a few ulps: with `f32`
positions, root offsets −5000 and 4999.99951171875 m and an animation translation of 0.7 mm (its
world position rounds to 5000 m) give 10000.0009765625 m
(`reexpressed_translations_round_past_twice_the_frame_by_ulps`). Nothing is refused for it.

`new` does not check that `map` can turn the chains of the neutral poses. Neutral poses far apart
put their distance into the transforms between the skeletons, so a mapped chain lies that far from
the root offset, where the chain allowance above grows. With the root offsets at opposite edges of
the frame, the first `map` of the ragdoll's neutral pose turns a half-metre chain 2 `MAX_POSITION`
from the root offset: accepted with `f32` positions, refused as `DegenerateChain` with the
`double-precision` feature (`neutral_poses_are_validated`).

## Six-DOF translation limits

Jolt corrects a violated limit by the distance beyond it times the effective mass
(`SixDOFConstraint.cpp:380-410,780-790`); limits within `MAX_SHAPE_EXTENT` keep that finite. A limit
of 1e30 m moved two parts to NaN positions in a few steps.

## Collision steps

`PhysicsWorld::MAX_COLLISION_STEPS` is 4. It bounds the jobs one update creates, because Jolt keeps
them in fixed-size storage: the job free list of the thread pool and of joltc's callback job system
(`cMaxPhysicsJobs`, 2048, `PhysicsSettings.h:25`), the barrier ring the stepping thread drains
(`JobSystemWithBarrier::cMaxJobs`, 2048, `Core/JobSystemWithBarrier.h:71`), and the array of static
job handles (`StaticArray<JobHandle, cMaxPhysicsJobs>`, `PhysicsSystem.cpp:585`). A caller job
system's table holds every queued job until the update returns. Jolt queues each job exactly once,
when its dependency count reaches zero, so the number of `JobSystem::queue_job` calls in one update
is the number of jobs that update created.

Jolt fixes the job counts once per update (`PhysicsSystem.cpp:240-263`), with the effective
concurrency `C = min(job system concurrency, 32)` (`PhysicsUpdateContext.h:34,152`). Per collision
step it creates at most:

- 11 single jobs: UpdateBroadPhasePrepare, UpdateBroadPhaseFinalize, BuildIslandsFromConstraints,
  FinalizeIslands, ContactRemovedCallbacks, BodySetIslandIndex, PreIntegrateVelocity,
  PostIntegrateVelocity, ResolveCCDContacts, SoftBodyPrepare and StartNextStep, which the last
  collision step lacks;
- StepListeners `<= C`, DetermineActiveConstraints `<= C - 2`, ApplyGravity `<= C - 2`,
  FindCollisions `<= C`, SetupVelocityConstraints `<= C`, SolveVelocityConstraints `C`,
  IntegrateVelocity `<= C`, SolvePositionConstraints `C` (`:266-570`);
- with active soft bodies `1 + 2C` more (`:2745-2779`), with continuous collision detection `<= C`
  more (`:1726`).

At `C = 32` that is 360 jobs per collision step and `360·N - 1` per update: 1439 at `N = 4`, 1079 at
`N = 3`, 359 at one collision step. Jolt's single-threaded job system (`C = 1`) stays far below.
The one kind of job this count leaves out is a FindCollisions job Jolt spawns while the step runs
(`TrySpawnJobFindCollisions`, `:830-878`); call their number `R`.

A count `N` is accepted when `(360·N - 1) + 2·R <= 2048`, `R` being the largest respawn count
measured below, with a factor two of margin on it because the measurement covers one scene, not
every scene. That allows `R <= 304` at `N = 4` and `R <= 484` at `N = 3`.

The measurement (`a_saturated_update_at_max_collision_steps_fits_jolts_job_pool`): a floor and
2048 awake unit cubes in a 16 x 16 grid of columns of 8, discrete motion, no constraints, listeners
or soft bodies, 30 updates of 4 collision steps each, on a counting inline job system of concurrency
32 and on a counting Rayon pool of 31 threads (concurrency 32). The scene's static count is
`171·N - 1` (11 single jobs, DetermineActiveConstraints 1, ApplyGravity 30, FindCollisions 32,
SetupVelocityConstraints 1 and 32 each of SolveVelocity, IntegrateVelocity and SolvePosition per
collision step), 683 at `N = 4`. Every update of both job systems queued exactly 683 jobs, so the
largest respawn count was 0 for both, in three runs. The scene starts the full 32 FindCollisions
jobs, and Jolt spawns another one only when one of them has finished while body pairs are still
waiting, which these runs never hit. `N = 4` is therefore accepted. The test asserts the count
formula and the budget, under a timeout, because an exhausted pool spins instead of failing.

## Contact constraint capacity

`WorldSettings::MAX_CONTACT_CONSTRAINTS` stays below the count above which
`ContactConstraintManager::Init` asserts; a native compile-time check pins it.

## Contact settings

A `ContactListener` may change the settings Jolt resolves a contact with. Every setter of
`ContactSettings` and `SoftBodyContactSettings` checks its value and refuses one outside its range
with a `ContactSettingsError`, leaving the settings unchanged. The rules below that depend on the
contact (sensor bodies, the lever `R`) are checked again on the value a listener returns, against
the contact it was called for, because a listener can assign a value kept from another contact;
`docs/events.md` ("Rejected contact settings") says what happens to a value that fails.

### Friction and restitution

Combined friction follows the body rule, `0..=limits::MAX_FRICTION`. Jolt's default combined
friction of two bodies at the body bound is `sqrt(MAX_FRICTION²) = MAX_FRICTION`, so a listener
cannot give a contact more friction than two bodies already can, and the body bound's probe
(`friction_at_the_bound_keeps_contacts_finite`) covers it. Combined restitution follows the body
rule `0..=1`.

### Inverse mass and inertia scales

Jolt documents the scales as "0 = infinite mass, 1 = use original mass, 2 = body has half the mass"
and multiplies them into the inverse mass and the inverse inertia of each body for the contact
(`ContactConstraintManager.cpp:930-949`); the soft body solver does the same for the vertices and
the other body (`SoftBodyMotionProperties.cpp:205-215`). Every bound this crate derives for masses
and inertia (`limits::MIN_MASS`, the inertia conditioning floor) assumes the inverse mass and
inertia a body has; a scale above 1 would raise them past what those bounds were derived for. The
API therefore accepts at most 1: a contact can make a body heavier or immovable, never lighter. This
is a policy of this API, not a Jolt limit.

Jolt treats only an exact 0 as immovable (`ContactConstraintManager.cpp:917-918`,
`SoftBodyMotionProperties.cpp:167-169`). When no dynamic body of a rigid contact keeps a factor
above 0 (a dynamic body with 0 against a static or kinematic one, or two dynamic bodies with 0
each), Jolt creates no contact constraint and the bodies pass through each other. Any other factor
goes into the inverse effective mass, so a tiny one makes the body enormously heavy: with a 1 kg
cube landing at 8 m/s, a factor of 1e-36 on its inverse mass and inertia overflows the contact
impulse, the cube's position is NaN three steps later, and an asserts build stops on
`MotionProperties.h:233`. A cloth landing on a block with the vertex factor 0 and the block's factor
1e-35 goes NaN the same way, through `p = dv / (w1 + w2)` (`SoftBodyMotionProperties.cpp:763-788`).

A positive factor is therefore at least `limits::MIN_CONTACT_SCALE = MIN_MASS / MAX_MASS` (1e-9). It
turns a body of `MIN_MASS` into one of `MAX_MASS`, the spread of masses two bodies can already have,
and makes any accepted body at most `MAX_MASS / MIN_CONTACT_SCALE = 1e15` kg heavy for the contact.
At the velocity bounds such a contact needs an impulse of about `1e15 kg * 1000 m/s = 1e18` N s,
twenty orders of magnitude below `f32::MAX`; the onsets measured above lie 26 to 27 orders below the
floor. `contact_scales_at_their_floor_step_finitely` throws cubes and spheres of `MIN_MASS` and
`MAX_MASS` at a floor at the velocity bounds, discrete and with `MotionQuality::LinearCast`, and a
light cube at a heavy one, with the mass and inertia factors at the floor or 0;
`soft_body_contact_scales_at_their_floor_step_finitely` throws a cloth at blocks of both masses with
the vertex and block factors at the floor or 0. Both stay finite and assert-free in the asserts
build.

`MIN_CONTACT_SCALE` is written as the literal `1e-9`: the `f32` division `MIN_MASS / MAX_MASS`
rounds one step above it, which would refuse the round number callers type.

### Sensor contacts

Jolt starts a contact's `mIsSensor` as `body1.IsSensor() || body2.IsSensor()` and asserts that a
callback does not turn a contact with a sensor body into an ordinary one
(`ContactConstraintManager.cpp:915`, `:1194`, `:1466`, "Sensors cannot be converted into regular
bodies by a contact callback!"). `ContactSettings::set_is_sensor(false)` is refused for such a
contact; making an ordinary contact a sensor contact is allowed. The soft body path has no such
assertion, so `SoftBodyContactSettings::set_is_sensor` takes either value.

### Surface velocity

Jolt applies the relative surface velocity at the friction point as `v + ω × r1`, `r1` the friction
point relative to body 1's centre of mass (`ContactConstraintManager.cpp:164-168`), and feeds the
result into the friction constraint as a target velocity. Bounding `v` by
`limits::MAX_LINEAR_VELOCITY` and `ω` by `limits::MAX_ANGULAR_VELOCITY` alone would still allow
`|ω| · |r1|` of about 47 rad/s times a lever of up to 2 km (`limits::MAX_SHAPE_EXTENT`), some 9e4
m/s. The setters therefore also require, in `f64`,

    |v| + |ω| · R <= limits::MAX_LINEAR_VELOCITY

with `R` the largest distance from body 1's centre of mass to a contact point of the manifold, on
either body, computed once per callback from the manifold the listener sees. The friction point is
an average of those points, so `|r1| <= R`, and `|v + ω × r1| <= |v| + |ω| · |r1|` keeps the target
speed within the linear velocity bound bodies have. Setting one of the two velocities checks it
together with the current value of the other.

The bound is checked by `surface_velocities_are_bounded_alone_and_together` (a lever of 2000 m:
`|ω|` of 0.25 rad/s fills the bound alone, and the next `f32` above is refused) and exercised by
`a_conveyor_moves_a_resting_cube` (2 m/s moves a resting cube along the floor).
