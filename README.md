[![Demo Video: Using this plugin for digital piano playing under 6DoF binaural with headphone](https://img.youtube.com/vi/cx1Pjfy-89w/maxresdefault.jpg)](https://youtu.be/cx1Pjfy-89w?si=LoWtRU2xak0PPzYk)


<img width="1140" height="809" alt="Screenshot 2025-12-10 at 2 37 24 AM" src="https://github.com/user-attachments/assets/a95b7d00-6f8b-4204-8f0f-0c8b6d02b30d" />

What this App can be extended in the future with VR and HMD:

(using either the VR model or 360 photo for visual representation (front end), coupled with this plugin for auralization (back end), with two-way OSC communication between the plugin and the visual representation tool such as Unreal Engine)

1: With an HMD, combine any 360° photo taken at the same location where the Ambisonics impulse response was captured, allowing both visualization and auralization of that space.

2: If you don’t have the 360-photo taken in a real set, but a virtual model instead. In this case, you can use geometrical virtual acoustics software like ODEON to generate a virtual ambisonics Impulse response signal for your model, which can then be imported into this plugin for playback.

3: If developed further on the VR side, the VR immersive mixing can be enabled, where the users can use their VR controller to move different instruments track object (movable meshes) in the VR scenes. As these VR objects are moved, their changing positional data is continuously sent back to the plugin for real-time auralization.

