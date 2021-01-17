# High-fidelity Models
This folder contains the `quadrotor_agent` class, which we use to simulate quadrotor dynamics evolving on SO(3).

Check out [this tiny repo](https://github.com/skousik/quadrotor-simulator-example) for an example of how to use this class.

This folder also contains a variety of low-level (tracking) controllers, which can be swapped out in the `quadrotor_agent` by changing its `LLC` property. We found that a tracking controller based on Taeyoung Lee's [geometric tracking control](https://ieeexplore.ieee.org/abstract/document/5717652?casa_token=BA7I3882uZIAAAAA:BUj4J-Wncz9PJ_xXPe3Hy0yCcaZN2uWR_eoVARdZ9BKpdDq38pnoT9tO1UqhcWggR2PM6oid) works best for tracking our parameterized RTD trajectories.
