---------------------------------------------------
---------------------------------------------------

HOW TO OPEN AN OSIM MODEL IN C++

---------------------------------------------------
---------------------------------------------------

1. Go to "C:\Users\SKUNNATH\Documents\OpenSim\4.5-2024-01-10-3b63585\Models\ <Open the file>
2. Make a new folder 'Geom'.
3. Copy the .osim file to the Geom file.
4. Go to "C:\OpenSim 4.5-2024-01-10-3b63585\Geometry" and copy all files to the new Geom file created.
5. The .osim files refer to these vtp files for importing geometry to these files.
6. Make changes in the directory for the model importing line of .cpp code.

---------------------------------------------------
---------------------------------------------------

VERIFICATION

---------------------------------------------------
---------------------------------------------------
The lines 17 and 18 of the .cpp file output the number of bodies in the model and list the names 
of the bodies in the model. This corresponds to the body names and count in Opensim.