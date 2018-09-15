# Getting started
**ncollide** relies on the official Rust package manager
[Cargo](https://crates.io) for dependency resolution and compilation. Therefore,
making **ncollide** ready to be used by your project is simply a matter of
adding a new dependency to your `Cargo.toml` file. You can either use the **ncollide2d**
crate for 2D geometry or the **ncollide3d** crate for 3D geometry. You can even use both
if you need both 2D and 3D in your application. Note that you will probably
need **nalgebra** as well because it defines algebraic entities
(vectors, points, transformation matrices) used by most types of **ncollide**.

```toml
[dependencies]
nalgebra = "0.16.0"
# Choose the one you need, or both.
ncollide2d = "0.17.0"
ncollide3d = "0.17.0"
```

Until **ncollide** reaches 1.0, it is strongly recommended to always use its
latest version, though you might encounter breaking changes from time to time.
Once your `Cargo.toml` file is set up, the corresponding crate must be imported
by your project with the usual `extern crate` directive:
```rust
extern crate ncollide2d; // If you need 2D.
extern crate ncollide3d; // If you need 3D.
```

## Cargo example
You may use this `Cargo.toml` file to compile the downloadable examples of this
guide. Simply replace `example.rs` by the actual example's file name.

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#cargo_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#cargo_3D">3D example</a></li>
</ul>

<div class="tab-content" markdown="1">
  <div id="cargo_2D" class="tab-pane in active">
```toml
[package]
name    = "example-using-ncollide"
version = "0.0.0"
authors = [ "You" ]

[dependencies]
approx   = "0.3.0"
alga     = "0.7.0"
nalgebra = "0.16.0"
ncollide2d = "0.17.0"

[[bin]]
name = "example"
path = "./example.rs"
```
  </div>
  <div id="cargo_3D" class="tab-pane">
```toml
[package]
name    = "example-using-ncollide"
version = "0.0.0"
authors = [ "You" ]

[dependencies]
approx   = "0.3.0"
alga     = "0.7.0"
nalgebra = "0.16.0"
ncollide3d = "0.17.0"

[[bin]]
name = "example"
path = "./example.rs"
```
  </div>
</div>