echo " - Downloading EuRoc dataset"
mkdir EuRoc
cd EuRoc
wget --execute="robots = off" --mirror --convert-links --no-parent --accept bag  http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/ 