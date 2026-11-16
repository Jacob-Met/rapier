import {
    init,
    Vector3,
    World,
    RigidBodyDesc,
    ColliderDesc,
} from "../builds/3d-deterministic/pkg";

describe("3d/World", () => {
    let world: World;

    beforeAll(init);

    afterAll(async () => {
        await Promise.resolve();
    });

    beforeEach(() => {
        world = new World(new Vector3(0, 9.8, 0));
    });

    afterEach(() => {
        world.free();
    });

    test("constructor", () => {
        expect(world.colliders.len()).toBe(0);
    });

    test("step skips wrapper rescans when the world has no soft bodies", () => {
        expect(world.softBodies.len()).toBe(0);
        const map = jest.spyOn(world, "mapNewSoftBodies");
        world.step();
        expect(map).not.toHaveBeenCalled();
    });

    test("step still maps topology changes when soft bodies exist", () => {
        const rawSoftBodies = jest
            .spyOn(world.softBodies.raw, "len")
            .mockReturnValue(1);
        const map = jest
            .spyOn(world, "mapNewSoftBodies")
            .mockImplementation(() => {});
        world.step();
        expect(map).toHaveBeenCalledTimes(1);
        rawSoftBodies.mockRestore();
        map.mockRestore();
    });

    test("removeRigidBody only unmaps its own colliders without soft bodies", () => {
        const other = world.createRigidBody(RigidBodyDesc.fixed());
        const otherCollider = world.createCollider(
            ColliderDesc.cuboid(1, 1, 1),
            other,
        );
        const removed = world.createRigidBody(RigidBodyDesc.dynamic());
        const removedCollider = world.createCollider(
            ColliderDesc.cuboid(1, 1, 1),
            removed,
        );
        const scan = jest.spyOn(world.colliders, "unmapRemovedColliders");

        world.removeRigidBody(removed);

        expect(scan).not.toHaveBeenCalled();
        expect(world.bodies.get(removed.handle)).toBeNull();
        expect(world.colliders.get(removedCollider.handle)).toBeNull();
        expect(world.bodies.get(other.handle)).toBe(other);
        expect(world.colliders.get(otherCollider.handle)).toBe(otherCollider);
    });

    test("removeRigidBody keeps soft-body cascade reconciliation", () => {
        const body = world.createRigidBody(RigidBodyDesc.dynamic());
        const softBodies = jest
            .spyOn(world.softBodies.raw, "len")
            .mockReturnValue(1);
        const scan = jest.spyOn(world.colliders, "unmapRemovedColliders");
        world.removeRigidBody(body);
        expect(scan).toHaveBeenCalledTimes(1);
        softBodies.mockRestore();
    });
});
