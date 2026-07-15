import matplotlib.pyplot as plt
import numpy as np
import math
def draw_arm_diagram1(L1, L2, theta1, theta2, target=None, save_path=None):
    fig, ax = plt.subplots(figsize=(6, 6))
 
    # Base position
    base = np.array([0, 0])
 
    # Joint 2 (elbow) position
    elbow = base + L1 * np.array([np.cos(theta1), np.sin(theta1)])
 
    # End-effector position
    end = elbow + L2 * np.array([np.cos(theta1 + theta2), np.sin(theta1 + theta2)])
 
    # --- Draw links ---
    ax.plot([base[0], elbow[0]], [base[1], elbow[1]], 'o-', lw=4, color='#2563eb', label='Link 1 (a1)')
    ax.plot([elbow[0], end[0]], [elbow[1], end[1]], 'o-', lw=4, color='#16a34a', label='Link 2 (a2)')
 
    # --- Joints ---
    ax.plot(*base, 'ks', markersize=12, zorder=5)  # base (square)
    ax.plot(*elbow, 'ko', markersize=10, zorder=5)  # elbow joint
    #ax.plot(*end, 'r*', markersize=18, zorder=5, label='End-effector')
 
    # --- Angle arcs (theta1 at base) ---
    arc1 = np.linspace(0, theta1, 50)
    r1 = 0.3
    ax.plot(r1 * np.cos(arc1), r1 * np.sin(arc1), color='gray', lw=1)
    ax.text(r1 * 1.4 * np.cos(theta1 / 2), r1 * 1.4 * np.sin(theta1 / 2),
            r'$\theta_1$', fontsize=13)
 
    # --- Reference dashed line (x-axis from base) ---
    ax.plot([0, L1 * 1.2], [0, 0], '--', color='gray', lw=0.8)
 
    # --- Angle arc theta2 at elbow ---
    arc2 = np.linspace(theta1, theta1 + theta2, 50)
    r2 = 0.3
    ax.plot(elbow[0] + r2 * np.cos(arc2), elbow[1] + r2 * np.sin(arc2), color='gray', lw=1)
    ax.text(elbow[0] + r2 * 1.4 * np.cos(theta1 + theta2 / 2),
            elbow[1] + r2 * 1.4 * np.sin(theta1 + theta2 / 2),
            r'$\theta_2$', fontsize=13)
 
    # --- Reference dashed line at elbow (parallel to link 1 direction) ---
    ref_end = elbow + L2 * 0.5 * np.array([np.cos(theta1), np.sin(theta1)])
    ax.plot([elbow[0], ref_end[0]], [elbow[1], ref_end[1]], '--', color='gray', lw=0.8)
 
    # --- Optional: show target point ---
    if target is not None:
        ax.plot(*target, 'x', color='crimson', markersize=12, mew=2, label='Target')
 
    # --- Labels ---
    ax.annotate('Base (origin)', base, textcoords="offset points", xytext=(-10, -20), fontsize=9)
    ax.annotate('Elbow joint', elbow, textcoords="offset points", xytext=(5, -10), fontsize=9)
    ax.annotate('End-effector', end, textcoords="offset points", xytext=(5, 5), fontsize=9)
    ax.annotate('a1', ((base[0] + elbow[0]) / 2, (base[1] + elbow[1]) / 2), textcoords="offset points", xytext=(5, -5), fontsize=9)
    ax.annotate('a2', ((elbow[0] + end[0]) / 2, (elbow[1] + end[1]) / 2), textcoords="offset points", xytext=(5, -5), fontsize=9)
    ax.set_xlim(-1, L1 + L2 + 1)
    ax.set_ylim(-1, L1 + L2 + 1)
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('2-Joint Planar Arm — Geometric IK Fig. 0')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(alpha=0.3)
 
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        print(f"Saved to {save_path}")
    plt.show()


    # --- Draw Diageram 2, angle Phi, law of cosines, triangle ---
 
def draw_arm_diagram2(L1, L2, theta1, theta2, target=None, save_path=None):
    fig, ax = plt.subplots(figsize=(6, 6))
 
    # Base position
    base = np.array([0, 0])
 
    # Joint 2 (elbow) position
    elbow = base + L1 * np.array([np.cos(theta1), np.sin(theta1)])
 
    # End-effector position
    end = elbow + L2 * np.array([np.cos(theta1 + theta2), np.sin(theta1 + theta2)])
 
    # --- Draw links ---
    ax.plot([base[0], elbow[0]], [base[1], elbow[1]], 'o-', lw=4, color='#2563eb', label='Link 1 (a1)')
    ax.plot([elbow[0], end[0]], [elbow[1], end[1]], 'o-', lw=4, color='#16a34a', label='Link 2 (a2)')
 
    # --- Joints ---
    ax.plot(*base, 'ks', markersize=12, zorder=5)  # base (square)
    ax.plot(*elbow, 'ko', markersize=10, zorder=5)  # elbow joint
    #ax.plot(*end, 'r*', markersize=18, zorder=5, label='End-effector')
 
    # --- Angle arcs (theta1 at base) ---
    arc1 = np.linspace(0, theta1, 50)
    r1 = 0.3
    ax.plot(r1 * np.cos(arc1), r1 * np.sin(arc1), color='gray', lw=1)
    ax.text(r1 * 1.4 * np.cos(theta1 / 2), r1 * 1.4 * np.sin(theta1 / 2),
            r'$\theta_1$', fontsize=13)
 
    # --- Reference dashed line (x-axis from base) ---
    ax.plot([0, L1 * 1.2], [0, 0], '--', color='gray', lw=0.8)
 
    # --- Angle arc theta2 at elbow ---
    arc2 = np.linspace(theta1, theta1 + theta2, 50)
    r2 = 0.3
    ax.plot(elbow[0] + r2 * np.cos(arc2), elbow[1] + r2 * np.sin(arc2), color='gray', lw=1)
    ax.text(elbow[0] + r2 * 1.4 * np.cos(theta1 + theta2 / 2),
            elbow[1] + r2 * 1.4 * np.sin(theta1 + theta2 / 2),
            r'$\theta_2$', fontsize=13)
    # --- Angle arc phi at elbow (between link 1 and link 2) ---
    """phi = np.pi - theta2  # internal angle at elbow
    arc_phi = np.linspace(theta1 + theta2, theta1 + np.pi, 50)
    r_phi = 0.2
    ax.plot(elbow[0] + r_phi * np.cos(arc_phi), elbow[1] + r_phi * np.sin(arc_phi), color='gray', lw=1)
    ax.text(elbow[0] + r_phi * 1.4 * np.cos(theta1 + phi / 2),
            elbow[1] + r_phi * 1.4 * np.sin(theta1 + phi / 2),
            r'$phi$', fontsize=13, color='gray')"""
 
    # --- Reference dashed line at elbow (parallel to link 1 direction) ---
    ref_end = elbow + L2 * 0.5 * np.array([np.cos(theta1), np.sin(theta1)])
    ax.plot([elbow[0], ref_end[0]], [elbow[1], ref_end[1]], '--', color='gray', lw=0.8)
 
    # --- Optional: show target point ---
    if target is not None:
        ax.plot(*target, 'x', color='crimson', markersize=12, mew=2, label='Target')

    # --- Math annotations for law of cosines ---
    mathtext = r'$\theta_2 = cos^{-1}\left(\dfrac{x^2+y^2-(a1)^2-(a2)^2}{2a1(a2)}\right)$'
    ax.text(2, 3, mathtext, fontsize=10, color='black', ha='right')
 
    # --- Labels ---
    ax.annotate('Base (origin)', base, textcoords="offset points", xytext=(-10, -20), fontsize=9)
    ax.annotate('Elbow joint', elbow, textcoords="offset points", xytext=(5, -10), fontsize=9)
    ax.annotate('End-effector', end, textcoords="offset points", xytext=(5, 5), fontsize=9)
    ax.annotate(r'$a1$', ((base[0] + elbow[0]) / 2, (base[1] + elbow[1]) / 2), textcoords="offset points", xytext=(5, -5), fontsize=9)
    ax.annotate(r'$a2$', ((elbow[0] + end[0]) / 2, (elbow[1] + end[1]) / 2), textcoords="offset points", xytext=(5, -5), fontsize=9)
    ax.fill([base[0], elbow[0], end[0]], [base[1], elbow[1], end[1]], color='orange', alpha=0.3, label=None)
    ax.annotate(r'$c$', ((base[0] + end[0]) / 2, (base[1] + end[1]) / 2), textcoords="offset points", xytext=(-5, 5), fontsize=9)
    ax.set_xlim(-1, L1 + L2 + 1)
    ax.set_ylim(-1, L1 + L2 + 1)
    ax.set_aspect('equal')
    ax.set_xlabel(r'$X$')
    ax.set_ylabel(r'$Y$')
    ax.set_title('2-Joint Planar Arm — Geometric IK Fig. 1')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(alpha=0.3)
 
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        print(f"Saved to {save_path}")
    plt.show()

def draw_arm_diagram3(L1, L2, theta1, theta2, target=None, save_path=None):
    fig, ax = plt.subplots(figsize=(6, 6))
 
    # Base position
    base = np.array([0, 0])
 
    # Joint 2 (elbow) position
    elbow = base + L1 * np.array([np.cos(theta1), np.sin(theta1)])
 
    # End-effector position
    end = elbow + L2 * np.array([np.cos(theta1 + theta2), np.sin(theta1 + theta2)])
 
    # --- Draw links ---
    ax.plot([base[0], elbow[0]], [base[1], elbow[1]], 'o-', lw=4, color='#2563eb', label='Link 1 (a1)')
    ax.plot([elbow[0], end[0]], [elbow[1], end[1]], 'o-', lw=4, color='#16a34a', label='Link 2 (a2)')
 
    # --- Joints ---
    ax.plot(*base, 'ks', markersize=12, zorder=5)  # base (square)
    ax.plot(*elbow, 'ko', markersize=10, zorder=5)  # elbow joint
    #ax.plot(*end, 'r*', markersize=18, zorder=5, label='End-effector')

    mathtext = r'$tan(\gamma)=\dfrac{y}{x}$'
    ax.text(1, 3, mathtext, fontsize=10, color='black', ha='right')
 
    # --- Angle arcs (theta1 at base) ---
    arc1 = np.linspace(0, theta1, 50)
    r1 = 0.3
    ax.plot(r1 * np.cos(arc1), r1 * np.sin(arc1), color='gray', lw=1)
    ax.text(r1 * 1.4 * np.cos(theta1 / 2), r1 * 1.4 * np.sin(theta1 / 2),
            r'$\theta_1$', fontsize=13)
 
    # --- Reference dashed line (x-axis from base) ---
    ax.plot([0, L1 * 1.2], [0, 0], '--', color='gray', lw=0.8)
 
    # --- Angle arc theta2 at elbow ---
    arc2 = np.linspace(theta1, theta1 + theta2, 50)
    r2 = 0.3
    ax.plot(elbow[0] + r2 * np.cos(arc2), elbow[1] + r2 * np.sin(arc2), color='gray', lw=1)
    #ax.text(elbow[0] + r2 * 1.4 * np.cos(theta1 + theta2 / 2),
    #        elbow[1] + r2 * 1.4 * np.sin(theta1 + theta2 / 2),
    #        r'$\theta_2$', fontsize=13)
    
    # --- Angle arc Gamma ---
    gamma = np.arctan2(end[1] - base[1], end[0] - base[0])
    r_gamma = 1
    arc_gamma = np.linspace(0,gamma, 50)
    ax.plot(base[0] + r_gamma * np.cos(arc_gamma), base[1] + r_gamma * np.sin(arc_gamma), color='purple', lw=1)
    ax.text(base[0] + r_gamma * 1.2 * np.cos((gamma) / 2),
            base[1] + r_gamma * 1 * np.sin((gamma) / 2),
            r'$\gamma$', fontsize=13, color='purple')
 
    # --- Reference dashed line at elbow (parallel to link 1 direction) ---
    ref_end = elbow + L2 * 0.5 * np.array([np.cos(theta1), np.sin(theta1)])
    ax.plot([elbow[0], ref_end[0]], [elbow[1], ref_end[1]], '--', color='gray', lw=0.8)
 
    # --- Optional: show target point ---
    if target is not None:
        ax.plot(*target, 'x', color='crimson', markersize=12, mew=2, label='Target')
 
    # --- Labels ---
    ax.annotate('Base (origin)', base, textcoords="offset points", xytext=(-10, -20), fontsize=9)
    #ax.annotate('Elbow joint', elbow, textcoords="offset points", xytext=(5, -10), fontsize=9)
    ax.annotate('End-effector', end, textcoords="offset points", xytext=(5, 5), fontsize=9)
    #ax.annotate(r'$a1$', ((base[0] + elbow[0]) / 2, (base[1] + elbow[1]) / 2), textcoords="offset points", xytext=(5, -5), fontsize=9)
    #ax.annotate(r'$a2$', ((elbow[0] + end[0]) / 2, (elbow[1] + end[1]) / 2), textcoords="offset points", xytext=(5, -5), fontsize=9)
    ax.fill([base[0], end[0], end[0]], [base[1], base[1], end[1]], color='orange', alpha=0.3, label=None)
    ax.annotate(r'$x$', ((base[0] + end[0]) / 2, (base[1] + base[1]) / 2), textcoords="offset points", xytext=(0, -10), fontsize=9)
    ax.annotate(r'$y$', ((end[0] + end[0]) / 2, (base[1] + end[1]) / 2), textcoords="offset points", xytext=(5, 0), fontsize=9)
    ax.set_xlim(-1, L1 + L2 + 1)
    ax.set_ylim(-1, L1 + L2 + 1)
    ax.set_aspect('equal')
    ax.set_xlabel(r'$X$')
    ax.set_ylabel(r'$Y$')
    ax.set_title('2-Joint Planar Arm — Geometric IK Fig. 2')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(alpha=0.3)
 
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        print(f"Saved to {save_path}")
    plt.show()

def draw_arm_diagram4(L1, L2, theta1, theta2, target=None, save_path=None):
    fig, ax = plt.subplots(figsize=(6, 6))
 
    # Base position
    base = np.array([0, 0])
 
    # Joint 2 (elbow) position
    elbow = base + L1 * np.array([np.cos(theta1), np.sin(theta1)])
 
    # End-effector position
    end = elbow + L2 * np.array([np.cos(theta1 + theta2), np.sin(theta1 + theta2)])
 
    # --- Draw links ---
    ax.plot([base[0], elbow[0]], [base[1], elbow[1]], 'o-', lw=4, color='#2563eb', label='Link 1 (a1)')
    ax.plot([elbow[0], end[0]], [elbow[1], end[1]], 'o-', lw=4, color='#16a34a', label='Link 2 (a2)')
 
    # --- Joints ---
    ax.plot(*base, 'ks', markersize=12, zorder=5)  # base (square)
    ax.plot(*elbow, 'ko', markersize=10, zorder=5)  # elbow joint
    #ax.plot(*end, 'r*', markersize=18, zorder=5, label='End-effector')

    mathtext = r'$tan(\beta) = \dfrac{a2sin(\theta_2)}{a1+a2cos(\theta_2)}$'
    ax.text(1.5, 3, mathtext, fontsize=10, color='black', ha='right')
 
    # --- Angle arcs (theta1 at base) ---
    arc1 = np.linspace(0, theta1, 50)
    r1 = 0.3
    ax.plot(r1 * np.cos(arc1), r1 * np.sin(arc1), color='gray', lw=1)
    ax.text(r1 * 1.4 * np.cos(theta1 / 2), r1 * 1.4 * np.sin(theta1 / 2),
            r'$\theta_1$', fontsize=13)
 
    # --- Reference dashed line (x-axis from base) ---
    ax.plot([0, L1 * 1.2], [0, 0], '--', color='gray', lw=0.8)
 
    # --- Angle arc theta2 at elbow ---
    arc2 = np.linspace(theta1, theta1 + theta2, 50)
    r2 = 0.3
    #ax.plot(elbow[0] + r2 * np.cos(arc2), elbow[1] + r2 * np.sin(arc2), color='gray', lw=1)
    ax.text(elbow[0] + r2 * 1.1 * np.cos(theta1 + theta2 / 2),
            elbow[1] + r2 * 1.1 * np.sin(theta1 + theta2 / 2),
            r'$\theta_2$', fontsize=10)
    
    # --- Angle arc Gamma ---
    gamma = np.arctan2(end[1] - base[1], end[0] - base[0])
    r_gamma = 1
    arc_gamma = np.linspace(0,gamma, 50)
    #ax.plot(base[0] + r_gamma * np.cos(arc_gamma), base[1] + r_gamma * np.sin(arc_gamma), color='purple', lw=1)
    #ax.text(base[0] + r_gamma * 1.2 * np.cos((gamma) / 2),
    #        base[1] + r_gamma * 1 * np.sin((gamma) / 2),
    #        r'$\gamma$', fontsize=13, color='purple')

    # --- Angle arc Beta ----
    beta = gamma - theta1
    r_beta = 0.8
    arc_beta = np.linspace(theta1, gamma, 50)
    ax.plot(base[0] + r_beta * np.cos(arc_beta), base[1] + r_beta * np.sin(arc_beta), color='crimson', lw=1)
    ax.text(base[0] + r_beta * 1.2 * np.cos(theta1+(beta/2)),
            base[1] + r_beta * 1.2 * np.sin(theta1+(beta/2)),
            r'$\beta$', fontsize=13, color='crimson')
 
    # --- Reference dashed line at elbow (parallel to link 1 direction) ---
    ref_end = elbow + L2 * 0.5 * np.array([np.cos(theta1), np.sin(theta1)])
    ax.plot([elbow[0], ref_end[0]], [elbow[1], ref_end[1]], '--', color='gray', lw=0.8)
 
    # --- Optional: show target point ---
    if target is not None:
        ax.plot(*target, 'x', color='crimson', markersize=12, mew=2, label='Target')
 
    # --- Labels ---
    lengths = np.array([L2 * np.cos(theta2), L2 * np.sin(theta2)])
    point = np.array([(lengths[0] + L1) * np.cos(theta1), (lengths[0] + L1) * np.sin(theta1)])
    ax.annotate('Base (origin)', base, textcoords="offset points", xytext=(-10, -20), fontsize=9)
    #ax.annotate('Elbow joint', elbow, textcoords="offset points", xytext=(5, -10), fontsize=9)
    ax.annotate('End-effector', end, textcoords="offset points", xytext=(5, 5), fontsize=9)
    ax.annotate(r'$a1$', ((base[0] + elbow[0]) / 2, (base[1] + elbow[1]) / 2), textcoords="offset points", xytext=(5, -5), fontsize=9)
    ax.annotate(r'$a2$', ((elbow[0] + end[0]) / 2, (elbow[1] + end[1]) / 2), textcoords="offset points", xytext=(-15, 5), fontsize=9)
    ax.fill([base[0], end[0], point[0]], [base[1], end[1], point[1]], color='orange', alpha=0.3, label=None)
    ax.annotate(r'$a2cos(\theta_2)$', ((lengths[0]/2 + L1) * np.cos(theta1), (lengths[0]/2 + L1) * np.sin(theta1)), textcoords="offset points", xytext=(-0, -10), fontsize=9)
    ax.annotate(r'$a2sin(\theta_2)$', ((end[0] + point[0])/2,(end[1] + point[1])/2), textcoords="offset points", xytext=(3, 3), fontsize=9)
    ax.set_xlim(-1, L1 + L2 + 1)
    ax.set_ylim(-1, L1 + L2 + 1)
    ax.set_aspect('equal')
    ax.set_xlabel(r'$X$')
    ax.set_ylabel(r'$Y$')
    ax.set_title('2-Joint Planar Arm — Geometric IK Fig. 3')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(alpha=0.3)
 
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        print(f"Saved to {save_path}")
    plt.show()
 
if __name__ == "__main__":
    # Example: L1=2, L2=1.5, arm bent at 40° and 30°
    #draw_arm_diagram1(L1=2, L2=1.5, theta1=np.radians(40), theta2=np.radians(30),
    #                  target=None, save_path="arm_schematic.png")
    draw_arm_diagram2(L1=2, L2=1.5, theta1=np.radians(40), theta2=np.radians(30),
                      target=None, save_path="arm_schematic2.png")
    draw_arm_diagram3(L1=2, L2=1.5, theta1=np.radians(40), theta2=np.radians(30),
                     target=None, save_path="arm_schematic3.png")
    draw_arm_diagram4(L1=2, L2=1.5, theta1=np.radians(40), theta2=np.radians(30),
                     target=None, save_path="arm_schematic4.png")