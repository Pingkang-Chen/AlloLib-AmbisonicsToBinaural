<img width="1153" height="846" alt="Screenshot 2025-09-25 at 4 22 35 AM" src="https://github.com/user-attachments/assets/923e2047-771a-40e9-b53d-27009828e618" />

What I will be adding next for this app:

1: The speaker layout of AlloSphere (54.1) where I will need to implement distance simulation, and more immersive/surround sound speaker layouts (2.0, 4.0, 5.1, 7.1, 5.1.4, 7.1.4, etc.)

2: More acoustic environments of interest will be added.

3: The option of selection among different SOFA file for HRTF dataset, and user can import their own individualized HRTF dataset (if they have).

4: The option of selecting among different provided headphones EQs (like Beyerdynamic, Sennheiser, AKG, ect.), for headphone compensation, ensuring a flat frequency response when listening over headphones.

5: The Ambisonics rotation with OSC connection for controlling the visual head to move accordingly with head tracking.

6: The option of importing your own wanted speaker layouts, via a .JSON file, so, users can upload whatever speaker layout into this app for playback.

7: Option to import any Ambisonics impulse response signal, enabling simulation of custom listening spaces.

8: This App for Ambisonics is currently using the ‘virtual loudspeaker method’. However, recent studies have suggested to employ an alternative formulation which encodes the HRTF in the SH domain for improved binaural signal’s quality. 

What this App can be extended in the future with VR and HMD:

(using either the VR model or 360 photo for visual representation (front end), coupled with this plugin for auralization (back end), with two-way OSC communication between the plugin and the visual representation tool such as Unreal Engine)

1: With an HMD, combine any 360° photo taken at the same location where the Ambisonics impulse response was captured, allowing both visualization and auralization of that space.

2: If you don’t have the 360-photo taken in a real set, but a virtual model instead. In this case, you can use geometrical virtual acoustics software like ODEON to generate a virtual ambisonics Impulse response signal for your model, which can then be imported into this plugin for playback.

3: If developed further on the VR side, the VR immersive mixing can be enabled, where the users can use their VR controller to move different instruments track object (movable meshes) in the VR scenes. As these VR objects are moved, their changing positional data is continuously sent back to the plugin for real-time auralization.

