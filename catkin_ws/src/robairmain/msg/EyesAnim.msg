# The matrix for this frame
EyesMat frame

# The position of this frame in the animation (first is 0)
uint8 index

# The total number of frames in this animation
uint8 frame_count
uint8 frame_max=5

# A more-or-less unique id for the animation, to avoid problems when multiple
# animations are sent at the same time (can be a random number generated on the
# spot)
uint8 id

# The duration in milliseconds
uint32 duration
