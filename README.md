Reconstructing what this was about a couple of years later...
=============================================================

You need stanford_wbc, with a build/ directory that is properly
populated.

You need to configure playtao by telling it where to find
stanford_wbc:

    cmake .. -DWBC_DIR:path=/path/to/stanford_wbc/top_src_directory

Launch playtao by passing a SAI XML file that you wish to
visualize. There are a couple of examples at the top source directory,
for example (assuming you're inside build/ underneath the playtao
sources):

    ./playtao -s ../puma.xml

You can drag-click the mouse around in the window that appears (it is
not the best mapping from mouse motion to orientation, but it works).

Then start the Python (Tix) robot client from a separate terminal, and
tell it how many DOFs it has to instantiate. This depends on your
robot, but apparently I never got around to communicating this
information from playtao to the client... anyway.  For the Puma, tell
it there are 6 DOF (assuming you're at the top of the playtao
sources):

    ./robot_client.py 6

That pops up an array of sliders, where the leftmost column
corresponds to the positions to be displayed in the graphical output
of playtao.

The middle and rightmost columns must have been intended for some kind
of control and/or visualization of Jacobians and whatnot, but
apparently I never got around to that either.

The whole shebang was used for debugging SAI XML files.  That format
is not the prettiest, or smartest for that matter, but it did the job
and we were focussing on what can be done with inverse dynamics (as
opposed to how to nicely model and specify the kinematics and dynamics
of a robot).
