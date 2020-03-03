#! /bin/bash

echo "Integrating rust documentation to the website..."
cssfile=./docs/rustdoc/rustdoc.css
tmpcssfile=./docs/rustdoc/rustdoc.css.tmp

echo '@import url("//fonts.googleapis.com/css?family=Lato:400,700,900,400italic");' > $tmpcssfile
echo '@import url("//cdn.rawgit.com/piscis/github-fork-ribbon-css-bem/v0.1.22/dist/gh-fork-ribbon-bem.min.css");' >> $tmpcssfile
cat $cssfile >> $tmpcssfile
cat custom_flatly/css/bootstrap-custom.min2.css >> $tmpcssfile
cat custom_flatly/css/base2.css >> $tmpcssfile
cat custom_flatly/css/font-awesome-4.0.3.css >> $tmpcssfile
sed -i -e 's/margin-left:[ ]*230px;//g' $tmpcssfile
mv $tmpcssfile $cssfile

files2d=`find ./docs/rustdoc/ncollide2d ./docs/rustdoc/src/ncollide2d -name \*.html -printf '%p '`
files3d=`find ./docs/rustdoc/ncollide3d ./docs/rustdoc/src/ncollide3d -name \*.html -printf '%p '`
sidebar='<nav class="sidebar">'
sub='<nav class="sub">'
container='<div class="container">'
class3='<div id="hide_medium" class="col-md-3">'
class9='<div class="col-md-9">'
end_div='</div>'
footer='<section class="footer"></section>'
body='<body[^>]*>'
head='<head>'
favicon='<link rel="shortcut icon" href="/img/favicon.ico">'


function patch_file() {
    echo "Patching $1."
    sed -i -e "s#$head#${head}${favicon}#g" $1
    sed -i -e "s#$sidebar#${container}${class3}${sidebar}#g" $1
    sed -i -e "s#$sub#${end_div}${class9}${sub}#g" $1
    sed -i -e "s#$footer#${end_div}${end_div}${footer}#g" $1
    sed -i -e "s#</body>\|</html>##g" $1
    sed -i -e "s#</head>#${css}</head>#g" $1

    nav='
    <div id='"'"'nav_placeholder'"'"'> </div>
    <script src="/jquery.js"></script>
    <script>
      var the_footer;
      $.get("/'$2'/index.html", function(data) {
          data = data.split("../").join("/");
          data = data.split("..").join("/");
          var $data = $(data);
          $("div#nav_placeholder").prepend($data.find("#common_navbar"));
      });
    </script>'

    escaped_nav=`echo $nav | sed ':a;N;$!ba;s/\n/ /g'`
    sed -i -e "s%${body}%&${escaped_nav}%g" $1
    fileend='
    <script>var base_url = "../" + window.rootPath;</script>
    <script src="/js/highlight.pack.js"></script>
    <script src="/js/bootstrap-3.0.3.min.js"></script>
    <script src="/js/base.js"></script>

  </body>
  </html>'
    echo $fileend >> $1
}

for file in `echo $files2d`
do
  patch_file $file "rustdoc_ncollide2d"
done


for file in `echo $files3d`
do
  patch_file $file "rustdoc_ncollide3d"
done

echo "... integration done!"
