const API_URL = "http://127.0.0.1:5000/data";

let rawData = [];

const tableBody = document.getElementById("data-table-body");

async function getData() {
    console.log("getData()");
    const response = await fetch(API_URL);
    const json = await response.json();
    rawData = json
      .map(r => ({
        timestamp: Number(r.timestamp),
        value: Number(r.value)
      }));
    renderTable();
}

function renderTable() {
  tableBody.innerHTML = "";
  for (const item of rawData) {
    const tr = document.createElement("tr");

    const dateCell = document.createElement("td");
    const date = new Date(item.timestamp * 1000);
    dateCell.textContent = date.toLocaleString();

    const valueCell = document.createElement("td");
    valueCell.textContent = item.value;

    tr.appendChild(dateCell);
    tr.appendChild(valueCell);
    tableBody.appendChild(tr);
  }
}

getData();