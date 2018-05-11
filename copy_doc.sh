#! /bin/sh

out_dir=./docs/rustdoc
ncollide_dir=../rust-dev/ncollide-dev

echo "Generating the documentation..."
cd $ncollide_dir; cargo doc --no-deps
cd -
rm -rf docs/rustdoc
cp -r $ncollide_dir/target/doc $out_dir

echo "... documentation generated!"

./fix_rustdoc.sh
