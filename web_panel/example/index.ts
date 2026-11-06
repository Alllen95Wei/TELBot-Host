let CURRENT_DEVICE: HIDDevice | null = null;
let REPORT_TABLE: HTMLTableElement;
const F310_RAW_RANGES: (RawDataRange | null)[] = [
    null,
    { range: 256, offset: 128, reverse: false },
    null,
    { range: 256, offset: 128, reverse: true },
    null,
    { range: 256, offset: 128, reverse: false },
    null,
    { range: 256, offset: 128, reverse: true },
    { range: 128, offset: 0, reverse: false },
    { range: 256, offset: 128, reverse: false },
    null,
    null,
    null,
    null,
];
const F310_BUTTONS: string[] = ["A", "B", "X", "Y", "LB", "RB", "Back", "Start"];

interface RawDataRange {
    range: number,
    offset: number,
    reverse: boolean
}

document.addEventListener("DOMContentLoaded", async () => {
    REPORT_TABLE = document.getElementById("report-table") as HTMLTableElement;

    document.getElementById("connect-btn")?.addEventListener("click", async () => {
        if (await checkHIDPerm()) {
            console.log("At least one HID device is present.");
        } else {
            console.log("No HID devices authorized. Requesting...");
            await navigator.hid.requestDevice({ filters: [] });
        }
        setHIDDevices();
    })

    document.getElementById("forget-btn")?.addEventListener("click", async () => {
        if (CURRENT_DEVICE) {
            await CURRENT_DEVICE.close();
            CURRENT_DEVICE.forget().then(() => {
                console.log("Forgot a HID device.");
            }).catch(console.error);
            clearCurrentDevice();
        }
    })

    navigator.hid.addEventListener("disconnect", (event) => {
        console.log(`Lost connection to HID device: ${event.device.productName}`);
        clearCurrentDevice();
    })
})

async function checkHIDPerm() {
    return (await navigator.hid.getDevices()).length !== 0;
}

function setHIDDevices() {
    navigator.hid.getDevices().then((devices: HIDDevice[]) => {
        const device = devices[0];
        CURRENT_DEVICE = device;
        console.log(device);
        (document.getElementById("connected-device-text") as HTMLSpanElement).textContent = device.productName;
        // Enable "Forget" button
        document.getElementById("forget-btn")?.removeAttribute("disabled");

        device.addEventListener("inputreport", (event) => {
            // console.log(`Received input report from ${device.productName}.`);
            const data = new Uint8Array(event.data.buffer);
            // Display the data
            // console.log(data);
            if (!REPORT_TABLE.hasAttribute("data-initialized")) initiateTable(data.length);
            for (let i = 0; i < data.length; i++) {
                (document.getElementById(`report-raw-${i}`) as HTMLTableCellElement).textContent = data[i].toString();
            }
            if (device.productId === 49693) rawF310ToDisplay(data);  // F310
        });

        if (!device.opened) device.open().then();
    })
}

function initiateTable(length: number) {
    clearTable();
    const idxRow = document.getElementById("report-table-idx") as HTMLTableRowElement;
    const rawRow = document.getElementById("report-table-raw") as HTMLTableRowElement;
    const percentRow = document.getElementById("report-table-percnt") as HTMLTableRowElement;
    const btnRow = document.getElementById("report-table-btn") as HTMLTableRowElement;

    for (let i = 0; i < length; i++) {
        const idxCell = document.createElement("td");
        idxCell.textContent = i.toString();
        idxCell.id = `report-idx-${i}`;
        idxRow.appendChild(idxCell);

        const rawCell = document.createElement("td");
        rawCell.textContent = "---"; // Placeholder
        rawCell.id = `report-raw-${i}`;
        rawRow.appendChild(rawCell);

        const percentCell = document.createElement("td");
        percentCell.textContent = "---";
        percentCell.id = `report-percnt-${i}`;
        percentRow.appendChild(percentCell);
    }

    const btnCell = document.createElement("td");
    btnCell.textContent = "---";
    btnCell.colSpan = length;
    btnCell.id = "report-btn";
    btnRow.appendChild(btnCell);

    REPORT_TABLE.setAttribute("data-initialized", "true");
}

function clearTable() {
    const idxRow = document.getElementById("report-table-idx") as HTMLTableRowElement;
    const rawRow = document.getElementById("report-table-raw") as HTMLTableRowElement;
    const percentRow = document.getElementById("report-table-percnt") as HTMLTableRowElement;
    const btnRow = document.getElementById("report-table-btn") as HTMLTableRowElement;

    idxRow.innerHTML = "";
    rawRow.innerHTML = "";
    percentRow.innerHTML = "";
    btnRow.innerHTML = "";

    const idxCell = document.createElement("td");
    idxCell.textContent = "#";
    idxRow.appendChild(idxCell);

    const rawCell = document.createElement("td");
    rawCell.textContent = "RAW";
    rawRow.appendChild(rawCell);

    const percentCell = document.createElement("td");
    percentCell.textContent = "%";
    percentRow.appendChild(percentCell);

    const btnCell = document.createElement("td");
    btnCell.textContent = "BTN";
    btnRow.appendChild(btnCell);

    REPORT_TABLE.removeAttribute("data-initialized");
}

// Converts specific fields of raw F310 `Uint8Array` data to percentage
function rawF310ToDisplay(rawArray: Uint8Array) {
    for (let i = 0; i < rawArray.length; i++) {
        if (F310_RAW_RANGES[i]) {
            const { range, offset, reverse } = F310_RAW_RANGES[i] as RawDataRange;
            let percent = ((rawArray[i] - offset) / range) * 100;
            if (reverse) percent *= -1;
            (document.getElementById(`report-percnt-${i}`) as HTMLTableCellElement).textContent = percent.toFixed(2);
        } else if (i === 10) {
            const rawBtnState = rawArray[i].toString(2).padStart(F310_BUTTONS.length, "0");
            let btnStates: string[] = [];
            // console.log(rawBtnState);
            for (let j = 0; j < F310_BUTTONS.length; j++) {
                if (rawBtnState[j] === "1") btnStates.push(F310_BUTTONS[F310_BUTTONS.length - j - 1]);
            }
            if (btnStates.length === 0) {
                (document.getElementById("report-btn") as HTMLTableCellElement).textContent = "---";
            } else {
                btnStates.reverse();  // Reverse to match the order of F310_BUTTONS
                (document.getElementById("report-btn") as HTMLTableCellElement).textContent = btnStates.join(", ");
            }
        }
    }
}

function clearCurrentDevice() {
    (document.getElementById("connected-device-text") as HTMLSpanElement).textContent = "N/A";
    CURRENT_DEVICE = null;
    document.getElementById("forget-btn")?.setAttribute("disabled", "true");
    clearTable();
}