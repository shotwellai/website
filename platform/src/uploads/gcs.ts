import { randomUUID } from "node:crypto";
import { basename, extname } from "node:path";
import { Storage } from "@google-cloud/storage";

import { config } from "../config.js";

const storage = new Storage();

export type UploadFileMetadata = {
  id: string;
  originalName: string;
  contentType: string;
  sizeBytes: number;
  objectName: string;
  uploadUrl: string;
};

export type NewUploadFileInput = {
  originalName: string;
  contentType?: string;
  sizeBytes?: number;
};

function requireBucketName() {
  if (!config.uploads.bucketName) {
    throw new Error("GCS_UPLOAD_BUCKET is not configured.");
  }

  return config.uploads.bucketName;
}

function safeObjectSegment(value: string) {
  const cleaned = basename(value)
    .normalize("NFKD")
    .replace(/[^\w.\-()+ ]+/g, "-")
    .replace(/\s+/g, "-")
    .replace(/-+/g, "-")
    .replace(/^-+|-+$/g, "");

  return cleaned || "upload";
}

function buildUploadObjectName(input: {
  userId: string;
  uploadId: string;
  fileId: string;
  originalName: string;
}) {
  const extension = extname(input.originalName);
  const base = safeObjectSegment(input.originalName).slice(0, 140);
  const fallbackName = extension ? `${input.fileId}${extension}` : input.fileId;
  const fileName = base.includes(".") ? base : fallbackName;

  return [
    config.uploads.uploadPrefix,
    input.userId,
    input.uploadId,
    `${input.fileId}-${fileName}`
  ]
    .filter(Boolean)
    .join("/");
}

export async function createGcsUploadSessions(input: {
  userId: string;
  uploadId: string;
  files: NewUploadFileInput[];
  origin: string;
}): Promise<UploadFileMetadata[]> {
  const bucket = storage.bucket(requireBucketName());

  return Promise.all(
    input.files.map(async (file) => {
      const id = randomUUID();
      const originalName = file.originalName.trim() || "upload";
      const contentType = file.contentType?.trim() || "application/octet-stream";
      const objectName = buildUploadObjectName({
        userId: input.userId,
        uploadId: input.uploadId,
        fileId: id,
        originalName
      });

      const [uploadUrl] = await bucket.file(objectName).createResumableUpload({
        origin: input.origin,
        metadata: {
          contentType,
          metadata: {
            shotwellUploadId: input.uploadId,
            originalName
          }
        }
      });

      return {
        id,
        originalName,
        contentType,
        sizeBytes: Math.max(0, Math.trunc(file.sizeBytes ?? 0)),
        objectName,
        uploadUrl
      };
    })
  );
}

export function resultObjectName(uploadId: string, fileName: string) {
  return [config.uploads.resultsPrefix, uploadId, safeObjectSegment(fileName)]
    .filter(Boolean)
    .join("/");
}

export function createResultReadStream(objectName: string) {
  return storage.bucket(requireBucketName()).file(objectName).createReadStream();
}

export function createUploadReadStream(objectName: string, range?: { start: number; end: number }) {
  return storage.bucket(requireBucketName()).file(objectName).createReadStream(range);
}
