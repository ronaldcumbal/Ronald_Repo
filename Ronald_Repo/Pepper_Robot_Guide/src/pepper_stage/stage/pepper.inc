define pepperlaser ranger
(
  sensor
  (
    range [0 3]
    fov 240
    samples 180
  )
  size [ 0.1 0.1 0.1 ]
  pose [ 0.0 0.0 -1.0 0 ]
)

define pepper position
(
    size [0.425 0.48 1]

    block
    (
        points 3
        point[0] [0.5 1]
        point[1] [0 0.5]
        point[2] [0.5 0]
        z [0 1.2]
     ) 
     mass 20
     gui_nose 1
     drive "omni"

     pepperlaser()
)
