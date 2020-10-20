''' Craig Reynolds' original webpage :: http://www.red3d.com/cwr/boids/ '''
from manimlib.imports import *
import QuadTree as qt
import colorutils as cu


###########################
## setup for simulations ##
###########################

#for introducing a bit of noise to the boids
def rotation_matrix(angle):
    #rotation about the z axis by an angle
    mat = np.array([[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0., 0., 1.0]])
    return mat

class Boid(VGroup):
    CONFIG = {
        "size" : 0.1,
        "boid_color" : None,
        "maxSpeed" : 2.0,
        "maxForce" : 3.0,
        "friendzone" : 1.0, #this is the basic perception radius
        #turn off any of the rules here
        "alignment" : True,
        "cohesion" : True,
        "separation" : True,
        "noise" : True,
        #wrap around in the X or Y direcion.
        "wrapX" : False,
        "wrapY" : False
    }

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        #the boid don't need a position since VGroup already has a center. To get the position, just use get_center
        self.velocity = (np.random.random(3) - 0.5)
        self.velocity[2] = 0.0 #manim has vectors in 3D. If we let the third component nonzero, the norm will do crazy things!
        self.noise_probability = 0.5
        #setting to maxspeed if only noise is present. This will make the random walk look prettier!!
        if not self.alignment and not self.cohesion and not self.separation:
            self.set_maxSpeed()
        self.alignment_radius = self.friendzone
        self.cohesion_radius = self.friendzone
        self.separation_radius = self.friendzone/2.0

        #a list which keeps track of other boids withing the perception
        self.friends = []
        #a list keeping track of all the obstacles
        self.obstacles = []

        self.add_body()

        #the updator for each boid.
        #since boid ingerits VGroup, and boid animations are applied to the VGroup, the body automatically follows the group center.
        self.add_updater(lambda m, dt : m.update_position(dt))

    def add_body(self):
        body = self.get_body()
        body.set_height(self.size)
        body.move_to(self.get_center())

        if self.boid_color is None:
            #setting a random color
            hue = random.randint(0, 255)
            col = cu.Color(hsv=(hue, 1.0, 1.0))
            col = col.hex
        else:
            col = self.boid_color
        body.set_color(color=col)
        body.rotate(np.arctan2(self.velocity[1], self.velocity[0]))
        self.add(body)
        self.body = body

    #body is a manim polygon Mobject.
    #This is written as a separate function so that it is easy to inherit the boid class and rewrite just the get_body function to get a new avatar!
    def get_body(self):
        c = self.get_center()
        v1 = c + RIGHT
        v2 = c + (UL + LEFT)
        v3 = c + (DL + LEFT)
        return Polygon(v1, v2, v3, fill_opacity=1.0)

    #getting other boids within the perception. Optimised using a QuadTree
    def get_friends(self, qtree):
        points = qtree.query_radius(self.get_center()[:-1], self.friendzone, [])
        for p in points:
            if p.payload != self:
                self.friends.append(p.payload)

    #for clearing the list of friends after each updation step
    def unfriend(self):
        self.friends = []

    #function to set to max speed
    def set_maxSpeed(self):
        speed = np.linalg.norm(self.velocity)
        if speed > 0.0:
            self.velocity *= self.maxSpeed / speed

    #incase repulsion from top and bottom boundaries is required(wrapy is false)
    def get_vertical_fear(self):
        d_top = FRAME_Y_RADIUS - self.get_center()[1]
        d_bot = +FRAME_Y_RADIUS + self.get_center()[1]
        #using MED_SMALL_BUFF = 0.25 as the fear radius
        if  0.0 < d_top < MED_LARGE_BUFF:
            return np.array([0., - 2. * self.maxForce/d_top, 0.])
        elif 0.0 < d_bot < MED_LARGE_BUFF:
            return np.array([0., 2. * self.maxForce/d_bot, 0.])
        else: return 0.0

    #incase repulsion from left and right boundaries is required(wrapy is false)
    def get_horizontal_fear(self):
        d_right = FRAME_X_RADIUS - self.get_center()[0]
        d_left = +FRAME_X_RADIUS + self.get_center()[0]
        #using MED_SMALL_BUFF = 0.25 as the fear radius
        if  0.0 < d_right < MED_LARGE_BUFF:
            return np.array([- 2. * self.maxForce/d_right, 0., 0.])
        elif 0.0 < d_left < MED_LARGE_BUFF:
            return np.array([2. * self.maxForce/d_left, 0., 0.])
        else: return 0.0

    #the end is the beginning!! #anything goes to the right comes from the left(and vice versa)!
    def wrapx(self):
        x, y = self.get_center()[:-1]
        if x > FRAME_X_RADIUS:
            self.move_to(np.array([-FRAME_X_RADIUS, y, 0.0]))
        elif x < - FRAME_X_RADIUS:
            self.move_to(np.array([FRAME_X_RADIUS, y, 0.0]))

    #the end is the beginning!! anything goes to the top comes from the bottom(and vice versa)!
    def wrapy(self):
        x, y = self.get_center()[:-1]
        if y > FRAME_Y_RADIUS:
            self.move_to(np.array([x, -FRAME_Y_RADIUS, 0.0]))
        elif y < - FRAME_Y_RADIUS:
            self.move_to(np.array([x, FRAME_Y_RADIUS, 0.0]))

    #Craig Reynold's algorithm step-1 :: Alignment. Link to the webpage on top
    def get_alignment(self):
        steering = np.zeros(3)
        desired_vel = np.zeros(3)
        total = 0.
        for friend in self.friends:
            d = np.linalg.norm(self.get_center() - friend.get_center())
            if d > 0.0 and d < self.alignment_radius:
                desired_vel += friend.velocity/d
                total += 1
        if total > 0:
            desired_vel /= total
            desired_speed = np.linalg.norm(desired_vel)
            if desired_speed > 0.0:
                desired_vel = desired_vel/desired_speed * self.maxSpeed
            steering = desired_vel - self.velocity
            if np.linalg.norm(steering) > 0.0:
                steering = steering / np.linalg.norm(steering) * self.maxForce
        return steering

    #Craig Reynold's algorithm step-2 :: Cohesion. Link to the webpage on top
    def get_cohesion(self):
        steering = np.zeros(3)
        center_of_mass = np.zeros(3)
        total = 0.
        for friend in self.friends:
            d = np.linalg.norm(self.get_center() - friend.get_center())
            if d > 0.0 and d < self.cohesion_radius:
                center_of_mass += friend.get_center()
                total += 1
        if total > 0:
            center_of_mass /= total
            steering = center_of_mass - self.get_center()
            if np.linalg.norm(steering) > 0.0:
                steering = steering / np.linalg.norm(steering) * self.maxForce
        return steering*0.5

    #Craig Reynold's algorithm step-3 :: Separation. Link to the webpage on top
    def get_separation(self):
        steering = np.zeros(3)
        total = 0.
        for friend in self.friends:
            force = self.get_center() - friend.get_center()
            d = np.linalg.norm(force)
            if d > 0.0 and d < self.separation_radius:
                steering += force/d
                total += 1
        if total > 0:
            steering /= total
            if np.linalg.norm(steering) > 0.0:
                steering = steering / np.linalg.norm(steering) * self.maxForce
        return steering

    #repulsion towards obstacles
    def get_obstacle_fear(self):
        mypos = self.get_center()
        steering = np.zeros(3)
        for obstacle in self.obstacles:
            d_top = np.linalg.norm(mypos - obstacle.get_top())
            d_bottom = np.linalg.norm(mypos - obstacle.get_bottom())
            d_right = np.linalg.norm(mypos - obstacle.get_right())
            d_left = np.linalg.norm(mypos - obstacle.get_left())
            fear_radius = self.friendzone
            #repels to all four sides
            if 0.0 < d_top < fear_radius:
                steering += (mypos - obstacle.get_top())/d_top
            if 0.0 < d_bottom < fear_radius:
                steering += (mypos - obstacle.get_bottom())/d_bottom
            if 0.0 < d_right < fear_radius:
                steering += (mypos - obstacle.get_right())/d_right
            if 0.0 < d_left < fear_radius:
                steering += (mypos - obstacle.get_left())/d_left
        if np.linalg.norm(steering) > 0.0:
                steering = steering / np.linalg.norm(steering) * self.maxForce
        return 2.0 * steering

    #averaging the color from the list of friends withing the perception.
    def update_color(self):
        color_list = [self.body.get_color()]
        for friend in self.friends:
            color_list.append(friend.body.get_color())
        newcolor = average_color(*color_list)
        return newcolor

    def update_position(self, dt):
        #adding up forces.
        force = np.zeros(3)
        if self.alignment:
            force += self.get_alignment()
        if self.cohesion:
            force += self.get_cohesion()
        if self.separation:
            force += self.get_separation()
        if not self.wrapX:
            force += self.get_horizontal_fear()
        if not self.wrapY:
            force += self.get_vertical_fear()
        force += self.get_obstacle_fear()

        #starting heading of the boid
        angle = np.arctan2(self.velocity[1], self.velocity[0])
        #updating the velocity
        self.velocity += force * dt
        #limiting the speed to maxSpeed
        speed = np.linalg.norm(self.velocity)
        if speed > self.maxSpeed:
            self.velocity *= self.maxSpeed / speed

        #noise is random deviations in the direction of the velocity.
        if self.noise and np.random.random() < self.noise_probability:
            ang = (np.random.random() - 0.5) * 2 * PI/12.
            rot = rotation_matrix(ang)
            self.velocity = np.dot(rot, self.velocity)

        #shifting the postion.
        self.shift(self.velocity * dt)

        #getting the angle to which the heading must be rotated
        angle = np.arctan2(self.velocity[1], self.velocity[0]) - angle
        self.rotate(angle)

        #wrapping around if required
        if self.wrapX:
            self.wrapx()
        if self.wrapY:
            self.wrapy()

        #interpolating colors
        self.body.set_color(self.update_color())
        #getting rid of the friends
        self.unfriend()

class DotBoid(Boid):
    def get_body(self):
        return Dot(fill_opacity=1.0)

class Obstacle(VGroup):
    CONFIG = {
        "type" : "circ",
        "size" : 0.2,
        "color" : RED
    }

    def __init__(self, x=None, y=None, **kwargs):
        super().__init__(**kwargs)
        if x is not None and y is not None:
            self.pos = np.array([x, y, 0.])
        else:
            self.pos = self.get_center()
        self.add_body()

    def add_body(self):
        if self.type == "circ":
            body = Circle(radius=self.size, color=self.color, fill_opacity=1.0)
        elif self.type == "rect":
            body = Rectangle(width=self.size, height=self.size, color=self.color, fill_opacity=1.0)
        body.move_to(self.pos)
        self.add(body)
        self.body = body

    def get_size(self):
        return self.size

class Flock(VGroup):
    CONFIG = {
        "num_boids" : 10,
        "boid_type" : Boid,
        "boid_config" : {"size" : 0.1,
                        "boid_color" : None,
                        "maxSpeed" : 3.0,
                        "maxForce" : 5.0,
                        "friendzone" : 1.0,
                        "alignment" : True,
                        "cohesion" : True,
                        "separation" : True,
                        "noise" : True,
                        "wrapX" : True,
                        "wrapY" : False},
    }

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.add_boids()
        self.obstacles = VGroup()
        self.add_updater(lambda m, dt: m.update_boids(dt))

    def add_boids(self):
        boids = VGroup()
        #randomising boid's positions
        for _ in range(self.num_boids):
            x = np.random.random() * (FRAME_WIDTH) - FRAME_X_RADIUS - MED_SMALL_BUFF
            y = np.random.random() * (FRAME_HEIGHT) - FRAME_Y_RADIUS - MED_SMALL_BUFF
            boid = self.boid_type(**self.boid_config).move_to(np.array([x, y, 0]))
            boids.add(boid)

        #the flock object has a VGroup containing all boids, and each boid is a VGroup!!
        self.boids = boids
        self.add(self.boids)

    def add_obstacles(self, obstacles):
        self.obstacles = obstacles
        self.add_to_back(obstacles)
        for boid in self.boids:
            for obs in self.obstacles:
                    boid.obstacles.append(obs)

    def update_boids(self, dt):
        #creates a new quadtree each step
        self.create_quadtree(self.boids)
        #each boid just need a friendlist, and then it will do its own thing!
        for boid in self.boids:
            boid.get_friends(self.qtree)

    #making the quadtree
    def create_quadtree(self, flock):
        boundary = qt.Rect(0., 0., FRAME_WIDTH, FRAME_HEIGHT)
        qtree = qt.QuadTree(boundary)
        for boid in flock:
            x, y = boid.get_center()[:-1]
            p = qt.Point(x, y, payload=boid)
            qtree.insert(p)
        self.qtree = qtree

#general setup for every flocking scene. Inherit this scene and give new CONFIG to make required scenes
class GeneralFlockScene(Scene):
    CONFIG = {
        #the default value of random seed is zero. So if we do not change this, we will
        #get the same scene over and over again for every run! We need this for the explanation scenes.
        "random_seed" : None, 
        "runtime" : 10,
        "obstacle_list" : [],
        "num_boids" : 10,
        "boid_color" : None,
        "boid_type" : Boid,
        "boid_size" : 0.07,
        "maxSpeed" : 3.0,
        "maxForce" : 5.0,
        "friendzone" : 0.7,
        "alignment" : True,
        "cohesion" : True,
        "separation" : True,
        "noise" : True,
        "wrapX" : True,
        "wrapY" : False
        }
    
    def setup(self):
        self.flock_config = {
                            "num_boids" : self.num_boids,
                            "boid_type" : self.boid_type,
                            "boid_config" : {
                                        "size" : self.boid_size,
                                        "boid_color" : self.boid_color,
                                        "maxSpeed" : self.maxSpeed,
                                        "maxForce" : self.maxForce,
                                        "friendzone" : self.friendzone,
                                        "alignment" : self.alignment,
                                        "cohesion" : self.cohesion,
                                        "separation" : self.separation,
                                        "noise" : self.noise,
                                        "wrapX" : self.wrapX,
                                        "wrapY" : self.wrapY
                                }
                            }
        self.create_flock()
        self.add_obstacles()
        self.add(self.flock)

    def create_flock(self):
        self.flock = Flock(**self.flock_config)

    def add_obstacles(self):
        self.flock.add_obstacles(VGroup(*self.obstacle_list))

    def construct(self):
        self.wait(self.runtime)

#################
#  simulations  #
#################



class Random_Walking_Boids(GeneralFlockScene):
    CONFIG = {
        "num_boids" : 10,
        "runtime" : 20,
        "alignment" : False,
        "cohesion" : False,
        "separation" : False,
        "noise" : True,
        "wrapX" : False
    }

class Only_Cohesion(GeneralFlockScene):
    CONFIG = {
        "num_boids" : 10,
        "runtime" : 20,
        "alignment" : False,
        "noise" : False,
        "separation" : False,
        "cohesion" : False,
        "wrapX" : False,
        "friendzone" : 2.0
    }

class Only_Separation(GeneralFlockScene):
    CONFIG = {
        "num_boids" : 10,
        "runtime" : 20,
        "alignment" : False,
        "noise" : False,
        "separation" : True,
        "cohesion" : False,
        "wrapX" : False,
        "friendzone" : 1.0
    }

class Only_Alignment(GeneralFlockScene):
    CONFIG = {
        "num_boids" : 10,
        "runtime" : 20,
        "alignment" : True,
        "noise" : False,
        "separation" : False,
        "cohesion" : False,
        "wrapX" : False,
        "friendzone" : 1.0
    }

class Cohesion_and_Separation(GeneralFlockScene):
     CONFIG = {
        "num_boids" : 10,
        "runtime" : 20,
        "alignment" : False,
        "noise" : False,
        "separation" : True,
        "cohesion" : True,
        "wrapX" : False,
        "friendzone" : 1.0
    }

class Cohesion_and_Alignment(GeneralFlockScene):
     CONFIG = {
        "num_boids" : 10,
        "runtime" : 20,
        "alignment" : True,
        "noise" : False,
        "separation" : False,
        "cohesion" : True,
        "wrapX" : False,
        "friendzone" : 1.0
    }

class Separation_and_Alignment(GeneralFlockScene):
     CONFIG = {
        "num_boids" : 10,
        "runtime" : 20,
        "alignment" : True,
        "noise" : False,
        "separation" : True,
        "cohesion" : False,
        "wrapX" : False,
        "friendzone" : 1.0
    }

obs_size = 0.5
xs = [obs_size * i for i in range(-3, 4, 1)]
obstacle_list = [Obstacle(x, 0, type='rect', size=obs_size, color=BLACK) for x in xs] #+ [Obstacle(x, -1.5, type='rect', size=obs_size, color=WHITE) for x in xs]
# obstacle_list = [Obstacle(0, 0, type='circ', size=0.5, color=WHITE)]
class For_End_Credits(GeneralFlockScene):
    CONFIG = {
        "num_boids" : 100,
        "obstacle_list" : obstacle_list,
        "runtime" : 20,
        "alignment" : True,
        "noise" : True,
        "separation" : True,
        "cohesion" : True,
        "wrapX" : True
    }
    # def construct(self):
    #     rect = Rectangle(height=obs_size, width=obs_size*7, fill_opacity=0.0, stroke_color=WHITE)
    #     self.add(rect)
    #     self.wait(self.runtime)

class Murmuration_Dots(GeneralFlockScene):
    CONFIG = {
        "boid_type" : DotBoid,
        "runtime" : 25,
        "boid_color" : BLACK,
        "boid_size" : 0.07,
        "num_boids" : 200,
        "camera_config" : {
            "background_color" : WHITE
        }
    }


############################################
#    Setup for Non-simulation animations   #
############################################

#this is a dummy boid object for non simualation animation. 
# The methods are gimmicks for making the animating process easier!!
class DummyBoid(VGroup):
    CONFIG = {
        "color" : RED, 
        "size" : 0.2,
        "perception" : 1.4
    }
    def __init__(self, x, y, **kwargs):
        super().__init__(**kwargs)
        self.velocity = (np.random.random(3) - 0.5) * 2.0       
        self.velocity[2] = 0.0
        self.add_body(x, y)

    def add_body(self, x, y):
        body = self.get_body()
        body.move_to(np.array([x, y, 0.0]))
        body.set_height(self.size)
        angle = np.arctan2(self.velocity[1], self.velocity[0])
        body.rotate(angle)
        self.add(body)
        self.body = body
        self.friends = []
    
    def get_body(self):
        c = self.get_center()
        v1 = c + RIGHT
        v2 = c + (UL + LEFT)
        v3 = c + (DL + LEFT)
        return Polygon(v1, v2, v3, color=self.color, fill_opacity=0.8)
    
    def change_velocity(self, vel):
        angle = np.arctan2(self.velocity[1], self.velocity[0])
        self.velocity = np.array(vel)
        angle = np.arctan2(self.velocity[1], self.velocity[0]) - angle
        self.rotate(angle)

    def get_friends(self, flock):
        for b in flock:
            d = np.linalg.norm(self.get_center() - b.get_center())
            if d < self.perception:
                self.friends.append(b)

class Boid_Rules_Template(ZoomedScene):
    #here, we are not changing the random seed! so we'll get the same cofiguration of boids for all teaching scenes!

    #create a flock of dummy boids
    def create_dummy_flock(self):
        flock = VGroup()
        for _ in range(40):
            x = np.random.random() * (FRAME_WIDTH) - FRAME_X_RADIUS - MED_SMALL_BUFF
            y = np.random.random() * (FRAME_HEIGHT) - FRAME_Y_RADIUS - MED_SMALL_BUFF
            b = DummyBoid(x, y, color=GREEN)
            flock.add(b)
        #highly inefficient. Basically making sure we have a region with 4 boids
        b = DummyBoid(5.0, -0.2, color=GREEN)
        flock.add(b)
        #chosen_one is the boid on which the explanations are performed
        self.chosen_one = b
        b2 = DummyBoid(5.7, 0.4, color=GREEN)
        b2.change_velocity([0.4, 0.8, 0.0])
        flock.add(b2)
        b3 = DummyBoid(4.2, -0.8, color=GREEN)
        b3.change_velocity([-0.4, 1.2, 0.0])
        flock.add(b3)
        self.flock = flock
    
    #general introduction scene
    def introduce_scene(self, animate=True):
        self.create_dummy_flock()
        r = self.chosen_one.perception
        self.perception_circ = Circle(radius=r, fill_opacity=0.1).move_to(self.chosen_one).set_color(GREY)
        self.chosen_one.get_friends(self.flock)
        not_friends = []
        for bd in self.flock:
            if bd not in self.chosen_one.friends and bd != self.chosen_one:
                not_friends.append(bd)
        
        if animate:
            self.play(AnimationGroup(*[GrowFromCenter(b) for b in self.flock], lag_ratio=0.0))
            self.wait(0.5)
            self.play(ApplyMethod(self.chosen_one.set_color, RED))
            self.play(GrowFromCenter(self.perception_circ)) 
            self.wait()
            self.play(
            self.camera_frame.set_width, self.perception_circ.get_width()*2.0, 
            self.camera_frame.move_to, self.chosen_one,
            AnimationGroup(*[FadeOut(boid) for boid in not_friends])
            )
            self.flock.remove(*not_friends)
            self.wait()
        else:
            self.add(self.flock)
            self.chosen_one.set_color(RED)
            self.camera_frame.set_width(self.perception_circ.get_width()*2.0)
            self.camera_frame.move_to(self.chosen_one)
            self.flock.remove(*not_friends)
            self.add(self.perception_circ)

    #another common method used in all teaching scenes
    def fade_friends(self, animate=True):
        if animate:
            animations = []
            for bd in self.chosen_one.friends:
                if bd != self.chosen_one:
                    animations.append(FadeOut(bd))
            return (AnimationGroup(*animations, lag_ratio=0.0))
        else:
            for bd in self.chosen_one.friends:
                if bd != self.chosen_one:
                    self.remove(bd)
            return None

    def bring_back_friends(self, animate=True):
        if animate:
            animations = []
            for bd in self.chosen_one.friends:
                if bd != self.chosen_one:
                    animations.append(FadeIn(bd))
            return (AnimationGroup(*animations, lag_ratio=0.0))
        else:
            for bd in self.chosen_one.friends:
                if bd != self.chosen_one:
                    self.add(bd)
            return None

    #general method to show the vector addition animation. returns the sum vector
    #zoom is just a fudge that we manually have to put in so that the summed arrow won't be outside the canvas
    def vector_addition(self, vectors, arrows, offset, zoom=2.0):
        animations = []
        s = vectors[0] + offset
        for i in range(1, len(arrows)):
            animations.append(ApplyMethod(arrows[i].move_to, s + vectors[i]/2.0))
            s += vectors[i]
        self.camera_frame.save_state()
        self.play(
            AnimationGroup(*animations, lag_ratio=0.2),
            self.camera_frame.set_width, 
            self.perception_circ.get_width()*zoom
            )
        self.wait()
        return s

    #the scaling of force and all that is shown here
    def teach_force(self, total_force, force_arrow, arrows, offset):
        self.play(
            AnimationGroup(
            ##Fade Friends
            self.fade_friends(),
            FadeOut(self.perception_circ),
            *[ApplyMethod(arr.set_opacity, 0.2) for arr in arrows]), 
            GrowArrow(force_arrow)
            )
        force_arrow.generate_target()
        start = offset
        end = offset + (total_force-offset)/(len(self.chosen_one.friends)-1)
        force_arrow.target.put_start_and_end_on(start=start, end=end).add_tip(tip_length=0.1)
        copy = force_arrow.copy().set_opacity(0.2)
        self.add(copy)
        self.wait(2)
        self.play(
            MoveToTarget(force_arrow), 
            self.camera_frame.restore
            )
        self.wait(0.5)
        self.play(FadeOut(arrows), FadeOut(copy))
        return (total_force-offset)/(len(self.chosen_one.friends)-1)

    def align_to_desired(self, force, tot_force_arrow, offset):
        b = self.chosen_one
        b_arrow = Line(offset, offset+b.velocity).add_tip(tip_length=0.1).set_color(RED)
        self.play(
            GrowArrow(b_arrow)
        )
        desired_v = b.velocity + force
        desired_arrow = Line(offset, offset + desired_v).add_tip(tip_length=0.1).set_color(TEAL)
        self.play(
            ApplyMethod(tot_force_arrow.move_to, offset + b.velocity + force/2.0),
        )
        self.play(
            GrowArrow(desired_arrow)
        )
        self.wait()
        self.play(
            self.bring_back_friends(),
            FadeIn(self.perception_circ)
        )
        self.wait()
        
        angle = np.arctan2(desired_v[1], desired_v[0]) - np.arctan2(b.velocity[1], b.velocity[0])
        self.play(
            ApplyMethod(b.rotate, angle),
            ReplacementTransform(b_arrow, desired_arrow),
            FadeOut(tot_force_arrow),
            run_time=1.5
        )

##################################
#   Non-Simulation Animations    #
##################################

class WhatIsPosition(VectorScene):
    def construct(self):
        boid = DummyBoid(2, 1)
        self.play(GrowFromCenter(boid))
        self.wait()
        self.add_plane(animate=True)
        self.bring_to_front(boid)
        o = Circle(color=BLUE, fill_opacity=1.0, radius=0.1)
        self.play(GrowFromCenter(o))
        

        xpos = DashedLine(ORIGIN, 2* RIGHT, color=YELLOW, stroke_width=8.0)
        xbrace = Brace(xpos, DOWN)
        xloc_text = TexMobject("x").next_to(xbrace, DOWN)
        ypos = DashedLine(2 * RIGHT, 2 * RIGHT + UP, color=YELLOW, stroke_width=8.0)
        ybrace = Brace(ypos, RIGHT)
        yloc_text = TexMobject("y").next_to(ybrace, RIGHT)
        self.wait()
        self.play(ShowCreation(xpos),  ShowCreation(ypos))
        self.play(GrowFromCenter(xbrace), GrowFromCenter(ybrace), Write(xloc_text), Write(yloc_text))
        self.bring_to_front(boid)
        self.wait()
        
        o_circ = Circle(color=YELLOW, radius=o.get_width()/2.0, fill_opacity=0.0)
        
        def surround_effect(mob, dt):
            op = mob.get_stroke_opacity()
            if op > 0.0:
                mob.set_stroke(opacity=op - 0.09)
                mob.set_width(mob.get_width() + 0.15)
            else:
                self.remove(mob)

        o_circ.add_updater(surround_effect)
        self.add(o_circ)
        self.wait(2)
        pos_vector = self.get_vector(boid.get_center(), color=YELLOW)
        self.play(
            ShrinkToCenter(xpos), 
            ShrinkToCenter(ypos), 
            ShrinkToCenter(xbrace), 
            ShrinkToCenter(ybrace), 
            ShrinkToCenter(xloc_text), 
            ShrinkToCenter(yloc_text)
            )
        self.wait(0.5)
        self.add_vector(pos_vector, run_time=1.5)
        self.bring_to_front(boid)
        self.wait(2)
        pos_vector.add_updater(lambda m: m.put_start_and_end_on(ORIGIN + 0.00001, boid.get_center()))
        self.play(ApplyMethod(boid.shift, RIGHT))
        self.wait(0.5)
        self.play(ApplyMethod(boid.shift, DL * 2.5))
        self.wait(0.5)
        self.play(ApplyMethod(boid.shift, UL * 3.5))
        self.wait(0.5)
        self.play(ApplyMethod(boid.shift, RIGHT * 5))       
        self.wait()

class VectorAddition(VectorScene):
    def construct(self):
        self.add_plane()
        vector_a = np.array([1.5, 1.0, 0.0])
        vector_b = np.array([2.0, - 0.5, 0.0])
        arrow_a = self.get_vector(vector_a, color=RED)
        arrow_b = self.get_vector(vector_b, color=GREEN)
        label_a = TexMobject("\\va{a}").set_color(RED)
        label_b = TexMobject("\\va{b}").set_color(GREEN)
        # self.add(arrow_a, arrow_b)
        self.play(GrowArrow(arrow_a))
        self.play(GrowArrow(arrow_b))
        ball = Circle(radius=0.4, fill_opacity=1.0)
        ball.set_color([YELLOW, WHITE])
        ball.set_sheen_direction(UL)
        self.play(FadeIn(ball))

        #wiggly effect
        self.play(ApplyMethod(arrow_a.rotate, PI/24, rate_func=wiggle))
        self.play(ApplyMethod(arrow_b.rotate, -PI/24, rate_func=wiggle))
        self.wait(0.5)
        vector_ab = vector_a + vector_b
        arrow_ab = self.get_vector(vector_ab)
        arrow_ab[0].set_color([RED, GREEN])
        arrow_ab[1].set_color(GREEN)
        arrow_ab.set_sheen_direction(RIGHT)
        label_ab = TexMobject("\\va{a}"," +",  " \\va{b}")
        label_ab[0].set_color(RED)
        label_ab[2].set_color(GREEN)

        label_ab.set_sheen_direction(RIGHT)
        self.bring_to_back(arrow_ab)
        # self.add(arrow_ab)
        self.play(FadeInFrom(arrow_ab, -vector_ab))
        ball.add_updater(lambda m, dt : m.move_to(m.get_center() + 2.4 * vector_ab * dt))
        self.add(ball)
        self.wait()
        self.play(FadeOut(arrow_ab))
        ball.clear_updaters()
        self.remove(ball)
        self.wait()
        arrow_a.save_state()
        self.play(ApplyMethod(arrow_a.move_to, vector_b + vector_a/2.0))
        self.play(GrowArrow(arrow_ab))
        label_a.move_to(arrow_a.get_center() + DOWN*0.5).scale(0.8)
        label_b.move_to(arrow_b.get_center() + DOWN*0.5).scale(0.8)
        label_ab.move_to(arrow_ab.get_center() + UP*0.5).scale(0.8)
        self.play(Write(label_a), Write(label_b), Write(label_ab))
        self.wait()
        
        self.play(FadeOut(label_a), FadeOut(label_b), FadeOut(label_ab))
        self.wait(0.5)
        self.play(ApplyMethod(arrow_ab.set_opacity, 0.2))
        self.play(ApplyMethod(arrow_a.restore)) 
        self.play(ApplyMethod(arrow_b.move_to, vector_a + vector_b/2.0))
        self.play(ApplyMethod(arrow_ab.set_opacity, 1.0))
        label_a.move_to(arrow_a.get_center() + 0.25*UL)
        label_b.move_to(arrow_b.get_center() + 0.5*UP)
        label_ab.move_to(arrow_ab.get_center() + 0.5*DOWN)
        self.play(Write(label_a), Write(label_b), Write(label_ab))
        self.wait()

class Adding_N_Vectors(VectorScene):
    def construct(self):
        self.add_plane()
        vectors = [np.array([1.5, -1.1, 0]),
                    np.array([0.8, 1.7, 0]),
                    np.array([-0.9, 1.9, 0]),
                    np.array([-0.3, -1.2, 0]),
                    np.array([-2.1, 0.2, 0.0])]
        arrows = VGroup()
        for vec in vectors:
            arr = self.get_vector(vec)
            # hue = random.randint(0, 255)
            col = interpolate_color(GREEN, RED, arr.get_length()/2.1)#cu.Color(hsv=(hue, 1, 1))
            arr.set_color(col)
            arrows.add(arr)

        self.play(AnimationGroup(*[GrowArrow(arr) for arr in arrows], lag_ratio=0.1))
        self.wait()
        animations = []
        s = vectors[0]
        for i in range(1, len(arrows)):
            animations.append(ApplyMethod(arrows[i].move_to, s + vectors[i]/2.0))
            s += vectors[i]


        self.play(AnimationGroup(*animations, lag_ratio=0.2))
        self.wait(0.5)
        s_arrow = self.get_vector(s)
        s_arrow.set_color(YELLOW)
        self.play(AnimationGroup(*[ApplyMethod(arr.set_opacity, 0.5) for arr in arrows]))
        self.add_vector(s_arrow)
        self.wait()

class Scaling_Vector(VectorScene):
    def construct(self):
        self.add_plane()
        vector = np.array([2.0, 1.5, 0.0])
        length = np.linalg.norm(vector)
        arrow = self.add_vector(vector, color=RED)
        label = TexMobject("1.0 \\times \\va{a}").move_to(arrow.get_center() + DR*0.5)
        self.play(Write(label))
        self.wait()
        def label_updater(mob):
            mob.generate_target()
            c = arrow.get_center()
            if c[0] < 0.0 and c[1] < 0.0:
                mult = "-" + str(round(arrow.get_length() / length * 100) / 100)
            else:
                mult = str(round(arrow.get_length() / length * 100) / 100)
            mob.target = TexMobject( mult + "\\times \\va{a}").move_to(arrow.get_center() + DR*0.5)
            mob.become(mob.target)
        label.add_updater(label_updater)
        self.add(label)

        arrow_copy = arrow.copy()

        self.play(
            ApplyMethod(arrow.put_start_and_end_on, ORIGIN + 0.00001, 2*vector), 
            run_time=2.0
            )
        self.wait()
        self.play(FadeIn(arrow_copy))
        self.play(
            ApplyMethod(arrow_copy.shift, UL/2)
        )
        self.wait()
        self.play(
            ApplyMethod(arrow.put_start_and_end_on, ORIGIN + 0.00001, 0.5*vector), 
            run_time=2.0
            )
        self.wait()
        self.play(
            ApplyMethod(arrow.put_start_and_end_on, ORIGIN + 0.00001, -1.0*vector), 
            run_time=2.0
            )
        self.wait()

class WhatIsCohesion(Boid_Rules_Template):
    def construct(self):
        #intro
        self.introduce_scene()
        #aliasing
        b = self.chosen_one
        c = b.get_center()

        #getting relative positions of other boids and corresponding arrows for animation.
        vectors = []
        arrows = VGroup()
        for bd in b.friends:
            if bd != b:
                vec = bd.get_center() - c
                vectors.append(vec)
                arrows.add(Line(c, bd.get_center()).add_tip(tip_length=0.1).set_color("#8B0000"))

        #animate those arrows
        self.play(AnimationGroup(*[GrowArrow(arr) for arr in arrows], lag_ratio=0.1))
        self.bring_to_front(b)
        self.wait(2)

        s = self.vector_addition(vectors, arrows, c)
        s_arrow = Line(c, s).add_tip(tip_length=0.1).set_color(DARK_BLUE)
        force = self.teach_force(s, s_arrow, arrows, c)
        self.wait()
        s_arrow_copy = s_arrow.copy()
        self.add(s_arrow_copy)
        text = TextMobject(" =", " Cohesion Force").scale(0.5)
        text[1].set_color(DARK_BLUE)
        self.play(ApplyMethod(s_arrow.shift, UL))
        text.next_to(s_arrow, RIGHT)
        self.play(Write(text))
        self.wait()
        self.play(
            Uncreate(text),
            Uncreate(s_arrow),
        )
        self.wait()
        self.align_to_desired(force, s_arrow_copy, c)
        self.wait()

class WhatIsSeparation(Boid_Rules_Template):
    def construct(self):
        #intro
        self.introduce_scene()
        self.wait()
        #aliasing
        b = self.chosen_one
        c = b.get_center()

        #getting realtive postions and corresponding arrows
        vectors = []
        arrows = VGroup()
        
        for bd in b.friends:
            if bd != b:
                vec = bd.get_center() - c
                vectors.append(vec)
                arrows.add(Line(c, bd.get_center()).add_tip(tip_length=0.1).set_color("#8B0000"))

        #steer_vec is to store all repulsive steering forces
        steer_vec = []

        #explaining the repulsion force for the first arrow
        arr = arrows[0].copy()
        self.play(GrowArrow(arr))
        self.wait(2)
        d = arr.get_length()
        steering = vectors[0]/d**2
        steer_arr = Line(c, c + steering).add_tip(tip_length=0.1).set_color("#8B0000")
        self.play(Transform(arr, steer_arr))
        self.wait(2)
        steering *= -1
        steer_arr = Line(c, c + steering).add_tip(tip_length=0.1).set_color(PURPLE)
        self.play(Transform(arr, steer_arr))
        self.wait(2)

        self.play(FadeOut(arr))
        b.save_state()
        self.play(
            AnimationGroup(*[FadeOut(bd) for bd in b.friends[1:] if bd != b], lag_ratio=0.0)
        )
        self.wait()
        self.why_divide_by_d()
        self.why_divide_by_d(UL + LEFT*0.2)
        self.play(
            b.restore      
        )
        self.play(
            AnimationGroup(*[FadeIn(bd) for bd in b.friends[1:] if bd != b ], lag_ratio=0.0)
        )

        #repeating the above procedure for all arrows, but quickly
        for i in range(0, len(vectors)):
            arr = arrows[i]
            self.play(GrowArrow(arr), run_time=0.4)
            d = arr.get_length()
            steering = -vectors[i]/d**2
            steer_arr = Line(c, c + steering).add_tip(tip_length=0.1).set_color(PURPLE)
            self.play(Transform(arr, steer_arr), run_time=0.4)
            steer_vec.append(steering)
            arrows[i].become(steer_arr)

        self.wait()

        s = self.vector_addition(steer_vec, arrows, c)
        s_arrow = Line(c, s).add_tip(tip_length=0.1).set_color("#0024c4")
        force = self.teach_force(s, s_arrow, arrows, c)
        self.wait()
        s_arrow_copy = s_arrow.copy()
        self.add(s_arrow_copy)
        text = TextMobject(" =", " Separation Force").scale(0.5)
        text[1].set_color("#0024c4")
        self.play(ApplyMethod(s_arrow.shift, UL))
        text.next_to(s_arrow, RIGHT)
        self.play(Write(text))
        self.wait()
        self.play(
            Uncreate(text),
            Uncreate(s_arrow),
        )
        self.wait()
        self.align_to_desired(force, s_arrow_copy, c)
        self.wait()
    
    #this is used twice, to show the effect of dividing by d
    def why_divide_by_d(self, new_pos=DR*0.5):
        b = self.chosen_one
        b1 = b.friends[0]
        self.play(ApplyMethod(b.shift, new_pos))
        displacement = b1.get_center() - b.get_center()
        temp_arr = Line(b.get_center(), b.get_center() + displacement).add_tip(tip_length=0.1).set_color("#8B0000")
        self.wait()
        self.play(GrowArrow(temp_arr))
        
        self.wait()
        direction_for_brace = np.dot(rotation_matrix(PI/2), displacement)
        brace = Brace(temp_arr, direction_for_brace)
        
        d = np.linalg.norm(displacement)
        dist_text = TexMobject("d = ", str(round(d*100)/100.)).move_to(brace.get_center() + 0.5*direction_for_brace).scale(0.4)
        angle = np.arctan2(displacement[1], displacement[0]) + PI
        dist_text.rotate(angle)
        self.play(
            GrowFromCenter(brace),
            Write(dist_text)
            )
        self.wait(3)
        self.play(
            Uncreate(dist_text),
            ShrinkToCenter(brace)
        )
        temp_arr.generate_target()
        temp_arr.target = Line(b.get_center(), b.get_center() + displacement/d**2).add_tip(tip_length=0.1).set_color("#8B0000")
        self.wait()
        self.play(MoveToTarget(temp_arr))
        temp_arr.generate_target()
        temp_arr.target = Line(b.get_center(), b.get_center() - displacement/d**2).add_tip(tip_length=0.1).set_color(PURPLE)
        self.wait()
        self.play(MoveToTarget(temp_arr))
        self.wait(2)
        self.play(FadeOut(temp_arr))
       
class WhatIsAlignment(Boid_Rules_Template):
    def construct(self):
        #intro
        self.introduce_scene(animate=False)
        #aliasing
        b = self.chosen_one
        c = b.get_center()
        self.wait()      
        #getting velocities of neighbors
        vel_vecs = []
        vel_arrows = VGroup()
        #velocity of the chosen boid
        b_arrow = Line(c, c+b.velocity).add_tip(tip_length=0.1).set_color(RED)
        for bd in b.friends:
            if bd != b:
                vel = bd.velocity/2
                vel_vecs.append(vel)
                cen = bd.get_center()
                arr = Line(cen, cen+vel).add_tip(tip_length=0.1).set_color("#32CD32")
                vel_arrows.add(arr)
        self.play(GrowArrow(b_arrow))
        self.wait()
        self.play(
            FadeOut(b_arrow), 
            AnimationGroup(*[GrowArrow(arr) for arr in vel_arrows])
            )
        #move the first velocity arrow to the chosen one's center
        self.wait()
        self.play(ApplyMethod(vel_arrows[0].move_to, c + vel_vecs[0]/2))

        s = self.vector_addition(vel_vecs, vel_arrows, c, zoom=2.5)
        s_arrow = Line(c, s).add_tip(tip_length=0.1).set_color(GOLD)
        force = self.teach_force(s, s_arrow, vel_arrows, c)
        self.wait(2)
        self.play(FadeIn(b_arrow))
        self.wait()

        #explain what is the alignment force
        steering = force - b.velocity
        steer_arrow = Line(c + b.velocity, c + b.velocity + steering).add_tip(tip_length=0.1).set_color(ORANGE)
        self.play(GrowArrow(steer_arrow))
        self.wait()
        self.play(
            FadeOut(b_arrow), 
            FadeOut(s_arrow), 
            ApplyMethod(steer_arrow.move_to, c+steering/2.0)
            )
        self.wait()
        steer_arrow_copy = steer_arrow.copy()
        self.add(steer_arrow_copy)
        text = TextMobject(" =", " Alignment Force").scale(0.5)
        text[1].set_color(ORANGE)
        self.play(ApplyMethod(steer_arrow.shift, UL))
        text.next_to(steer_arrow, RIGHT)
        self.play(Write(text))
        self.wait()
        self.play(
            Uncreate(text),
            Uncreate(steer_arrow),
        )
        self.wait()
        self.align_to_desired(steering, steer_arrow_copy, c)
        self.wait()

class TotalForces(Boid_Rules_Template):
    def construct(self):
        
        self.introduce_scene(False)
        b = self.chosen_one
        self.play(
            ApplyMethod(self.flock.shift, RIGHT*1.2), 
            ApplyMethod(self.perception_circ.shift, RIGHT*1.2)
        )
        # self.play(
        #     self.fade_friends(),
        #     FadeOut(self.perception_circ)
        #     )
        c = b.get_center()
        c_force = 0.0
        s_force = 0.0
        a_force = 0.0
        l = len(b.friends) - 1
        for bd in b.friends:
            if bd != b:
                a_force += bd.velocity/2
        a_force /= l
        a_force -= b.velocity

        for bd in b.friends:
            if bd != b:
                c_force += bd.get_center() - c
        c_force /= l

        for bd in b.friends:
            if bd != b:
                f = bd.get_center() - c
                s_force += -f/np.linalg.norm(f)**2
        s_force /= l

        left = self.camera_frame.get_left() + np.array([MED_LARGE_BUFF, 0.0, 0.0])
        top = UP/2.0
        c_arrow =  Line(c, c + c_force).add_tip(tip_length=0.1).set_color(DARK_BLUE)
        s_arrow =  Line(c, c + s_force).add_tip(tip_length=0.1).set_color("#0024c4")
        a_arrow =  Line(c, c + a_force).add_tip(tip_length=0.1).set_color(ORANGE)
        c_text = TextMobject("Cohesion").set_color(DARK_BLUE).scale(0.4)
        s_text = TextMobject("Separation").set_color("#0024c4").scale(0.4)
        a_text = TextMobject("Alignment").set_color(ORANGE).scale(0.4)
        tot_text = TextMobject("Total Force").set_color(MAROON).scale(0.4)
        plus1 = TexMobject("+").scale(0.4)
        plus2 = plus1.copy()
        equal = TexMobject("=").scale(0.4)
        equalc = equal.copy()
        equals = equal.copy() 
        equala = equal.copy()
        equalb = equal.copy()
        equalt = equal.copy()
        b_vel_text = TextMobject("Current Velocity").set_color(RED).scale(0.4)
        plus3 = plus1.copy()
        desired_text = TextMobject("Desired Velocity").set_color(GOLD).scale(0.4)
        # self.add(c_arrow, s_arrow, a_arrow, c_text, s_text, a_text)
        self.play(
            GrowArrow(c_arrow)
            )
        c_text.move_to(left + c_text.get_width()/2.0 + top)
        c_arrow_copy = c_arrow.copy()
        self.play(
            Write(c_text),
            Write(equalc.next_to(c_text, RIGHT)),
            ApplyMethod(c_arrow_copy.next_to, equalc, RIGHT)
            )
        self.wait()
        equals.next_to(equalc, DOWN)
        s_text.next_to(equals, LEFT)
        s_arrow_copy = s_arrow.copy()
        self.play(
            GrowArrow(s_arrow)
            )
        self.play(
            Write(s_text),
            Write(equals),
            ApplyMethod(s_arrow_copy.next_to, equals, RIGHT)
            )
        self.wait()
        equala.next_to(equals, DOWN)
        a_text.next_to(equala, LEFT)
        a_arrow_copy = a_arrow.copy()
        self.play(
            GrowArrow(a_arrow)
            )
        self.play(
            Write(a_text),
            Write(equala),
            ApplyMethod(a_arrow_copy.next_to, equala, RIGHT)
        )
        self.wait()
        
        self.play(
            FadeOut(a_text),
            FadeOut(s_text),
            FadeOut(c_text),
            FadeOut(equalc),
            FadeOut(equals),
            FadeOut(equala),
            ApplyMethod(c_arrow_copy.move_to, left),
            Write(plus1.move_to(left + 0.25*RIGHT)),
            ApplyMethod(s_arrow_copy.move_to, left+0.5*RIGHT),
            Write(plus2.move_to(left + 0.75*RIGHT)),
            ApplyMethod(a_arrow_copy.move_to, left+RIGHT)
        )
        self.wait()
        # self.play(FadeOut(c_text), FadeOut(s_text), FadeOut(a_text))
        s = a_force 
        self.play(ApplyMethod(s_arrow.move_to, c + s + s_force/2.0), run_time=0.2)
        s += s_force
        self.play(ApplyMethod(c_arrow.move_to, c + s + c_force/2.0), run_time=0.2)
        s += c_force

        total_arrow = Line(c, c + s).add_tip(tip_length=0.1).set_color(MAROON)
        self.wait(0.3)
        self.play(
            GrowArrow(total_arrow),
            ApplyMethod(c_arrow.set_opacity, 0.2),
            ApplyMethod(s_arrow.set_opacity, 0.2),
            ApplyMethod(a_arrow.set_opacity, 0.2),
            run_time=0.5
        )
        total_arrow_copy = total_arrow.copy()
        self.add(total_arrow_copy)
        self.play(
            Write(equal.move_to(a_arrow_copy.get_center() + 0.3*RIGHT)),
            ApplyMethod(total_arrow_copy.move_to, equal.get_center() + 0.4*RIGHT)
        )
        self.wait()
        getting_rid = VGroup(*[
            s_arrow_copy, 
            a_arrow_copy,
            c_arrow_copy,
            plus1,
            plus2
            ])
        tot_text.next_to(equal, LEFT)
        self.play(
            FadeOut(c_arrow),
            FadeOut(s_arrow),
            FadeOut(a_arrow),
            ReplacementTransform(getting_rid, tot_text)
            )
        self.wait()
        
        b_arrow = Line(c, c + b.velocity).add_tip(tip_length=0.1).set_color(RED)
        b_arrow_copy = b_arrow.copy()
        self.play(GrowArrow(b_arrow))
        self.add(b_arrow_copy)
        equalb.next_to(equal, UP)
        b_vel_text.next_to(equalb, LEFT)
        self.play(
            Write(b_vel_text),
            Write(equalb),
            ApplyMethod(b_arrow_copy.next_to, total_arrow_copy, UP)
            )
        self.wait()
        desired_v = b.velocity + s
        self.play(ApplyMethod(total_arrow.move_to, c + b.velocity + s/2.0))
        self.play(
            FadeOut(b_vel_text),
            FadeOut(tot_text),
            FadeOut(equalb),
            FadeOut(equal),
            ApplyMethod(b_arrow_copy.move_to, equal.get_center() + LEFT*1.5),
            Write(plus3.move_to(equal.get_center() + LEFT)),
            ApplyMethod(total_arrow_copy.move_to, equal.get_center() + LEFT*0.5)
        )
        desired_arrow = Line(c, c + desired_v).add_tip(tip_length=0.1).set_color(GOLD)
        desired_arrow_copy = desired_arrow.copy()
        self.play(GrowArrow(desired_arrow))
        self.add(desired_arrow_copy)
        self.play(
            Write(equal),
            ApplyMethod(desired_arrow_copy.next_to, equal, RIGHT)
            )
        getting_rid = VGroup(*[
            plus3,
            b_arrow_copy,
            total_arrow_copy
        ])
        desired_text.next_to(equal, LEFT)
        self.play(
            ReplacementTransform(getting_rid, desired_text)
        )
        self.wait()
        angle = np.arctan2(desired_v[1], desired_v[0]) - np.arctan2(b.velocity[1], b.velocity[0])
        self.play(
            ApplyMethod(b.rotate, angle),
            ReplacementTransform(b_arrow, desired_arrow),
            FadeOut(total_arrow),
            run_time=1.5
        )
        self.wait()
