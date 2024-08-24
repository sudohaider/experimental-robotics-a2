(define (problem task)
(:domain turtlebot)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    kenny - robot
)
(:init
    (robot_at kenny wp3)

    (visited wp4)
    (visited wp1)
    (visited wp2)
    (visited wp3)

    (undocked kenny)


    (localised kenny)

    (dock_at wp0)

    (= (charge kenny) 0)

)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
    (docked kenny)
    (>  (charge kenny) 0)
))
)
