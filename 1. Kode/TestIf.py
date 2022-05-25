estimated_pos = [-100,0]

if estimated_pos[0] >= 17.5:
    estimated_pos[0] = 17.5
elif estimated_pos[0] <= -17.5:
    estimated_pos[0] = -17.5

print(estimated_pos)
