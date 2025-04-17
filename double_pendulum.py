import sys, pygame
import numpy as np

pygame.init()

# Clock
clock = pygame.time.Clock()

# Screen
pygame.display.set_caption("Pendulum")
size = width, height = 1000, 600
screen = pygame.display.set_mode(size)

class Double_Pendulum():
    def __init__(self, masses, init_pos, init_vel, L):
        self.g = 9.81 # m/s**2
        self.b = 0.0 # damping

        self.mass1, self.mass2 = masses # kg
        self.L1, self.L2 = L # m
        self.theta01, self.theta02 = init_pos # gradians
        self.thetadot01, self.thetadot02 = init_vel # rad/s

        self.SCALE = 200 # px
        self.color = np.random.randint(0,255, size=3)

        self.path = []

        self.offsetx1, self.offsety1 = width//2, 25
        self.offsetx2, self.offsety2 = self.offsetx1 + np.sin(self.theta02), self.offsety1 + np.cos(self.theta02)
        
        self.S01 = [np.radians(self.theta01), self.thetadot01]
        self.S02 = [np.radians(self.theta02), self.thetadot02]

        self.S0 = self.S01 + self.S02
        self.t = 0.0
        self.k = 0

    def dSdt(self, S, t):
        theta1, thetadot1, theta2, thetadot2 = S
        
        delta = theta2 - theta1
        
        sdel= np.sin(delta)
        cdel= np.cos(delta)
        
        masstotal = self.mass1 + self.mass2


        alpha1_num = self.mass2 * self.L1 * thetadot1**2 * sdel * cdel + self.mass2*self.g*np.sin(theta2)*cdel + self.mass2*self.L2*thetadot2**2*sdel - masstotal*self.g*np.sin(theta1)
        alpha1_den = masstotal*self.L1 - self.mass2*self.L1*cdel**2

        alpha2_num = -self.mass2 * self.L2 * thetadot2**2 * sdel * cdel + masstotal*(self.g*np.sin(theta1)*cdel - self.L1*thetadot1**2*sdel - self.g*np.sin(theta2))
        alpha2_den = masstotal*self.L2 - self.mass2*self.L2*cdel**2

        return thetadot1, alpha1_num/alpha1_den, thetadot2, alpha2_num/alpha2_den

    def RK4(self, f, y, t, step):
        yn = y
        dt = step
        
        k1 = dt * np.array(f(y       , t       ))
        k2 = dt * np.array(f(y + k1/2, t + dt/2))
        k3 = dt * np.array(f(y + k2/2, t + dt/2))
        k4 = dt * np.array(f(y + k3  , t + dt  ))

        y = y + (k1 + 2*k2 + 2*k3 + k4)/6 

        return y

    def draw(self, S0):
        
        if len(self.path) >= 2:
            path = []
            for pp in self.path:
                xx, yy = pp
                path.append((xx, yy))

            for i in range(len(path)-1):
                pygame.draw.line(screen, self.color, path[i], path[i+1], 1)
            

        if len(self.path) > 1002:
            self.path.pop(0)
        
        # elif len(self.path) >= 2 and len(self.path) < 12:
        #     lw = 2
        #     pygame.draw.lines(screen, self.color, False, self.path, lw)
        # else:
        #     lw = 1

        #     pygame.draw.lines(screen, self.color, False, path, lw)

        x1 = self.offsetx1 + self.SCALE*self.L1*np.sin(S0[0])
        y1 = self.offsety1 + self.SCALE*self.L1*np.cos(S0[0])

        self.offsetx2, self.offsety2 = x1 + np.sin(self.theta02), y1 + np.cos(self.theta02)

        x2 = self.offsetx2 + self.SCALE*self.L2*np.sin(S0[2])
        y2 = self.offsety2 + self.SCALE*self.L2*np.cos(S0[2])
        
        self.path.append((x2, y2))

        pygame.draw.line(screen, self.color, (self.offsetx1, self.offsety1), (x1, y1), 2)
        pygame.draw.line(screen, self.color, (self.offsetx2, self.offsety2), (x2, y2), 2)

        pygame.draw.circle(screen, self.color, (x1, y1), self.mass1*np.pi)
        pygame.draw.circle(screen, self.color, (x2, y2), self.mass2*np.pi)


    def update(self, dt):
        h = 1/dt
        
        self.S0 = self.RK4(self.dSdt, self.S0, self.t, h)
        self.t = self.t + h

        self.draw(self.S0)

dp_list = []
for i in range(10):
    dp_list.append(Double_Pendulum((2, 2), (90+i*0.05, 90+i*0.05), (0, 0), (1+i*0.001, 1-i*0.001)))


tn = 0.0
font = pygame.font.SysFont("Tahoma", size=12)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    
    # Delta time
    dt = clock.tick(30)
    tn += 1/dt

    # Draw
    screen.fill((0,0,0))

    for dp_i in dp_list:
        dp_i.update(dt)
    

    # tn_min = round(tn//60, 1)
    # tn_sec = round(tn%60 , 1)

    # font_render = font.render(f"Time: {tn_min} min {tn_sec} sec", True, (255,255,255))

    # screen.blit(font_render, (10, 10))

    # Update
    pygame.display.update()