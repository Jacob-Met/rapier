import {init, Vector3, World} from "../builds/3d-deterministic/pkg";

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
        const rawSoftBodies = jest.spyOn(world.softBodies.raw, "len").mockReturnValue(1);
        const map = jest.spyOn(world, "mapNewSoftBodies").mockImplementation(() => {});
        world.step();
        expect(map).toHaveBeenCalledTimes(1);
        rawSoftBodies.mockRestore();
        map.mockRestore();
    });
});
